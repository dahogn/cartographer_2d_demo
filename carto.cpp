#include <map>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/mapping/map_builder.h>
#include <cartographer/io/image.h>
#include <cartographer/io/submap_painter.h>

#include "data_transform.h"
#include "simple_grid_map.h"
#include "carto.h"

using namespace cartographer;

CartoModule::CartoModule(char *dir, char *file, PoseUpdateCallback cb_function) : config_files_dir(dir), config_file_name(file), callback(cb_function)
{

    LOG(INFO) << "Initialize cartographer with configuration file " << config_files_dir << "/" << config_file_name;
    usleep(100000);
    auto file_resolver = absl::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{config_files_dir});
    const std::string code = file_resolver->GetFileContentOrDie(config_file_name);
    common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
    map_builder_options = mapping::CreateMapBuilderOptions(lua_parameter_dictionary.GetDictionary("map_builder").get());
    trajectory_builder_options = mapping::CreateTrajectoryBuilderOptions(lua_parameter_dictionary.GetDictionary("trajectory_builder").get());

    //创建MapBuilder
    map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(map_builder_options);
    //创建trajectory
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> sensor_ids;
    sensor_ids.insert(SensorId{SensorType::RANGE, range_name});
    if (using_imu)
    {
        sensor_ids.insert(SensorId{SensorType::IMU, imu_name});
    }

    trajectory_id = map_builder->AddTrajectoryBuilder(sensor_ids, trajectory_builder_options,
                                                      [this](const int id,
                                                             const ::cartographer::common::Time time,
                                                             const transform::Rigid3d local_pose,
                                                             sensor::RangeData range_data_in_local,
                                                             const std::unique_ptr<const ::cartographer::mapping::TrajectoryBuilderInterface::InsertionResult> res) {
                                                          OnLocalSlamResult2(id, time, local_pose, range_data_in_local);
                                                      });
    LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";
    //获取trajectory_builder
    trajectory_builder = map_builder->GetTrajectoryBuilder(trajectory_id);
    if (!trajectory_builder)
    {
        LOG(ERROR) << "Get Trajectory Builder Failed";
    }

    pthread_mutex_init(&mutex, nullptr);
    pthread_mutex_init(&sensor_mutex, nullptr);
};

CartoModule::~CartoModule()
{
    stop_and_optimize();
    pthread_mutex_destroy(&mutex);
    pthread_mutex_destroy(&sensor_mutex);
};

int CartoModule::handle_radar_data(PointCloudData &data)
{
    pthread_mutex_lock(&mutex);
    if (trajectory_id < 0)
    {
        pthread_mutex_unlock(&mutex);
        return 1;
    }
    // LOG(INFO)<<"Add range data at timestamp " << data.timestamp << " point count " << data.points.size();
    pthread_mutex_lock(&sensor_mutex);
    if (latest_sensor_timestamp < 0 || latest_sensor_timestamp < data.timestamp)
    {
        trajectory_builder->AddSensorData(range_name, to_carto_point_cloud(data, radar_scan_time));
        latest_sensor_timestamp = data.timestamp;
    }
    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_unlock(&mutex);
    return 0;
};

int CartoModule::handle_imu_data(ImuData2D &data)
{
    if (!using_imu)
    {
        return 1;
    }
    pthread_mutex_lock(&mutex);
    if (trajectory_id < 0)
    {
        pthread_mutex_unlock(&mutex);
        return 1;
    }
    // LOG(INFO)<<"Add imu data at timestamp " << data.timestamp_us << " " << data.linear_acceleration_x << " " << data.linear_acceleration_y;
    pthread_mutex_lock(&sensor_mutex);
    if (latest_sensor_timestamp < 0 || latest_sensor_timestamp < data.timestamp_us)
    {
        trajectory_builder->AddSensorData(imu_name, to_carto_imu(data));
        latest_sensor_timestamp = data.timestamp_us;
    }
    pthread_mutex_unlock(&sensor_mutex);
    pthread_mutex_unlock(&mutex);
    return 0;
};

void CartoModule::stop_and_optimize()
{
    pthread_mutex_lock(&mutex);
    if (trajectory_id < 0)
    {
        pthread_mutex_unlock(&mutex);
        return;
    }
    //结束trajectory
    map_builder->FinishTrajectory(trajectory_id);
    trajectory_id = -1;
    usleep(1000000);
    //运行最终的全局优化
    map_builder->pose_graph()->RunFinalOptimization();
    pthread_mutex_unlock(&mutex);
};

void CartoModule::paint_map(std::vector<char> *output)
{
    using io::PaintSubmapSlicesResult;
    using io::SubmapSlice;
    using mapping::SubmapId;

    MapInfo info;
    pthread_mutex_lock(&mutex);
    double resolution = 0.05;

    //获取所有子图的位姿
    std::map<SubmapId, SubmapSlice> submap_slices;
    auto submap_poses = map_builder->pose_graph()->GetAllSubmapPoses();
    for (const auto &submap_id_pose : submap_poses)
    {
        SubmapId submap_id = submap_id_pose.id;
        transform::Rigid3d pose = submap_id_pose.data.pose;
        int version = submap_id_pose.data.version;

        //查询子图内容
        mapping::proto::SubmapQuery::Response response_proto;
        const std::string error = map_builder->SubmapToProto(submap_id, &response_proto);
        if (!error.empty())
        {
            LOG(ERROR) << error;
            pthread_mutex_unlock(&mutex);
            return;
        }
        int query_version = response_proto.submap_version();
        if (response_proto.textures_size() == 0)
        {
            LOG(INFO) << "Responds of submap query is empty for submap '" << submap_id.submap_index << "'";
            continue;
        }
        //提取第一个Texture
        auto first_texture = response_proto.textures().begin();
        std::string cells = first_texture->cells();
        int width = first_texture->width();
        int height = first_texture->height();
        resolution = first_texture->resolution();
        // LOG(INFO) << "############resolution=" << resolution<< "##############";
        transform::Rigid3d slice_pose = transform::ToRigid3(first_texture->slice_pose());
        auto pixels = io::UnpackTextureData(cells, width, height);

        //填写SubmapSlice
        SubmapSlice &submap_slice = submap_slices[submap_id];
        submap_slice.pose = pose;
        submap_slice.metadata_version = version;
        submap_slice.version = query_version;
        submap_slice.width = width;
        submap_slice.height = height;
        submap_slice.slice_pose = slice_pose;
        submap_slice.resolution = resolution;
        submap_slice.cairo_data.clear();
        submap_slice.surface = ::io::DrawTexture(
            pixels.intensity, pixels.alpha, width, height,
            &submap_slice.cairo_data);
    } //for (const auto& submap_id_pose : submap_poses)

    pthread_mutex_unlock(&mutex);
    LOG(INFO) << "Get and draw " << submap_slices.size() << " submaps";

    //使用Submap绘制地图
    auto painted_slices = PaintSubmapSlices(submap_slices, resolution);
    int width = cairo_image_surface_get_width(painted_slices.surface.get());
    int height = cairo_image_surface_get_height(painted_slices.surface.get());

    info.width = width;
    info.height = height;
    info.origen_x = -painted_slices.origin.x() * resolution;
    info.origen_y = (-height + painted_slices.origin.y()) * resolution;
    info.resolution = resolution;

    SimpleGridMap *grid_map = new SimpleGridMap(info);

    uint32_t *pixel_data = reinterpret_cast<uint32_t *>(cairo_image_surface_get_data(painted_slices.surface.get()));
    grid_map->datas.reserve(width * height);
    for (int y = height - 1; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            const uint32_t packed = pixel_data[y * width + x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value = observed == 0 ? -1 : common::RoundToInt((1. - color / 255.) * 100.);
            CHECK_LE(-1, value);
            CHECK_GE(100, value);
            grid_map->datas.push_back((char)value);
        }
    }

    LOG(INFO) << "Paint map with width " << width << ", height " << height << ", resolution " << resolution;

    grid_map->to_char_array(output);
    delete grid_map;
};

void CartoModule::OnLocalSlamResult(
    const int id, const common::Time time,
    const transform::Rigid3d local_pose,
    sensor::RangeData range_data_in_local,
    const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> insertion_result)
{
    OnLocalSlamResult2(id, time, local_pose, range_data_in_local);
    return;
};

void CartoModule::OnLocalSlamResult2(
    const int id, const common::Time time,
    const transform::Rigid3d local_pose,
    sensor::RangeData range_data_in_local)
{
    transform::Rigid3d local2global = map_builder->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
    transform::Rigid3d pose3d = local2global * local_pose;

    Position pose2d = from_carto_rigid3d(from_carto_time(time), pose3d);
    callback(pose2d);

    // LOG(INFO) << "Local slam callback at " << from_carto_time(time) << " : " << pose3d.DebugString() << ". with landmark point " << landmark_count;

    return;
};
