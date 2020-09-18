#ifndef CARTO_MODULE_H
#define CARTO_MODULE_H

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <list>

#include <Eigen/Core>

#include <cartographer/common/fixed_ratio_sampler.h>
#include <cartographer/common/time.h>

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/pose_extrapolator.h>

#include "pose.h"
#include "sensor_data.h"

using namespace cartographer;

typedef void (*PoseUpdateCallback)(Position &);

class CartoModule
{
public:
    CartoModule(char *dir, char *file, PoseUpdateCallback cb_function);
    ~CartoModule();

    int handle_radar_data(PointCloudData &data);
    int handle_imu_data(ImuData2D &data);

    void paint_map(std::vector<char> *output);

    void stop_and_optimize();

private:
    bool using_imu = true;
    bool using_odom = false;
    std::string range_name = "range0";
    std::string imu_name = "imu0";
    std::string odom_name = "odom0";

    std::string config_files_dir;
    std::string config_file_name;
    double radar_scan_time = 0.0001;

    pthread_mutex_t sensor_mutex;
    long latest_sensor_timestamp = -1;

    pthread_mutex_t mutex;

    std::unique_ptr<mapping::MapBuilderInterface> map_builder;           //建图接口MapBuilder
    mapping::TrajectoryBuilderInterface *trajectory_builder;             //路径接口指针TrajectoryBuilder
    mapping::proto::MapBuilderOptions map_builder_options;               //MapBuilder参数
    mapping::proto::TrajectoryBuilderOptions trajectory_builder_options; //TrajectoryBuilder参数
    int trajectory_id;

    PoseUpdateCallback callback;

    void OnLocalSlamResult(
        const int trajectory_id, const common::Time time,
        const transform::Rigid3d local_pose,
        sensor::RangeData range_data_in_local,
        const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> insertion_result);
    void OnLocalSlamResult2(
        const int trajectory_id, const common::Time time,
        const transform::Rigid3d local_pose,
        sensor::RangeData range_data_in_local);
};

#endif
