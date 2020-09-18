#ifndef CARTO_SENSOR_DATA_H
#define CARTO_SENSOR_DATA_H

#include <vector>

#include <Eigen/Core>

#include "pose.h"

class ImuData2D
{
public:
    ImuData2D() : ImuData2D(0, 0., 0., 0.){};
    ImuData2D(long time_us, double angular_velocity) : ImuData2D(time_us, 0., 0., angular_velocity){};
    ImuData2D(long time_us,
              double acceleration_x,
              double acceleration_y,
              double angular_velocity)
        : timestamp_us(time_us),
          linear_acceleration_x(acceleration_x),
          linear_acceleration_y(acceleration_y),
          angular_velocity_z(angular_velocity){};

    long timestamp_us;
    double linear_acceleration_x;
    double linear_acceleration_y;
    double angular_velocity_z;
    double angle_z;             //已废弃

    void to_char_array(std::vector<char> *output);
    void from_char_array(char *buf, int size);
};

class PointCloudData
{
public:
    PointCloudData() : PointCloudData(0){};
    PointCloudData(long time_us) : timestamp(time_us){};
    PointCloudData(long time_us, int size) : timestamp(time_us)
    {
        points.reserve(size);
        intensities.reserve(size);
    };

    void add_point(double x, double y)
    {
        add_point(x, y, 0.25);
    };
    void add_point(double x, double y, double intensity)
    {
        points.push_back(Eigen::Vector2d(x, y));
        intensities.push_back((float)intensity);
    };

    long timestamp;
    std::vector<Eigen::Vector2d> points;
    std::vector<float> intensities;

    void to_char_array(std::vector<char> *output);
    void from_char_array(char *buf, int size);
};

#endif
