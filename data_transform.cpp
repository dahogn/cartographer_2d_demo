#include <cmath>

#include "data_transform.h"

Position from_carto_rigid3d(long time_us, cartographer::transform::Rigid3d &pose)
{

    Eigen::Matrix<double, 3, 1> trans = pose.translation();
    Eigen::Matrix3d rotation_matrix = pose.rotation().matrix();
    Eigen::Vector3d x_axis(1., 0., 0.);
    Eigen::Vector3d new_x_axis = rotation_matrix * x_axis;
    double x = new_x_axis(0);
    double y = new_x_axis(1);
    double theta = atan2(y, x);

    return Position(time_us, trans(0), trans(1), theta);
}

cartographer::transform::Rigid3d to_carto_ridig3d(Position &pose)
{

    Eigen::Matrix<double, 3, 1> trans;
    trans << pose.x, pose.y, 0;
    Eigen::Vector3d eulerAngle(pose.theta, 0, 0);
    Eigen::Quaterniond quaternion;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
    quaternion = yawAngle * pitchAngle * rollAngle;

    return cartographer::transform::Rigid3d{trans, quaternion};
}

ImuData2D from_carto_imu(cartographer::sensor::ImuData &imu)
{
    long timestamp_us = from_carto_time(imu.time);
    double linear_acceleration_x = imu.linear_acceleration(0);
    double linear_acceleration_y = imu.linear_acceleration(1);
    double angular_velocity_z = -imu.angular_velocity(2);
    return ImuData2D(timestamp_us, linear_acceleration_x, linear_acceleration_y, angular_velocity_z);
}

cartographer::sensor::ImuData to_carto_imu(ImuData2D &imu)
{

    return cartographer::sensor::ImuData{
        to_carto_time(imu.timestamp_us),
        Eigen::Vector3d(imu.linear_acceleration_x, imu.linear_acceleration_y, 9.8),
        Eigen::Vector3d(0., 0., -imu.angular_velocity_z)};
}

cartographer::sensor::TimedPointCloudData to_carto_point_cloud(PointCloudData &data, double time_offset)
{

    cartographer::common::Time time = to_carto_time(data.timestamp);
    cartographer::sensor::TimedPointCloud carto_data;
    int size = data.points.size();
    carto_data.reserve(size);
    double timestep = time_offset / size;
    double minus_time = -time_offset;
    for (int i = 0; i < size; i++)
    {
        if (i == size - 1)
        {
            minus_time = 0.;
        }
        cartographer::sensor::TimedRangefinderPoint tmp_point{
            Eigen::Vector3f(data.points.at(i)(0), data.points.at(i)(1), 0.), minus_time};
        carto_data.push_back(tmp_point);
        minus_time += timestep;
    }

    return cartographer::sensor::TimedPointCloudData{time, Eigen::Vector3f(0, 0, 0), carto_data};
}

long from_carto_time(cartographer::common::Time time)
{
    return (cartographer::common::ToUniversal(time) / 10);
}

cartographer::common::Time to_carto_time(long timestamp_us)
{
    return cartographer::common::FromUniversal(timestamp_us * 10);
}
