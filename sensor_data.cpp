#include "sensor_data.h"

void ImuData2D::to_char_array(std::vector<char> *output)
{
    output->reserve(output->size() + sizeof(ImuData2D));
    char *p = (char *)this;
    for (int i = 0; i < sizeof(ImuData2D); i++)
    {
        output->push_back(p[i]);
    }
}

void ImuData2D::from_char_array(char *buf, int size)
{
    if (size >= sizeof(ImuData2D))
    {
        ImuData2D *p = (ImuData2D *)buf;
        timestamp_us = p->timestamp_us;
        linear_acceleration_x = p->linear_acceleration_x;
        linear_acceleration_y = p->linear_acceleration_y;
        angle_z = p->angle_z;
        angular_velocity_z = p->angular_velocity_z;
    }
}

void PointCloudData::to_char_array(std::vector<char> *output)
{
    int size = sizeof(long) + sizeof(float) * points.size() * 3;
    char *buf = new char[size];
    char *p = buf;
    *((long *)p) = timestamp;
    p += sizeof(long);
    int num = points.size();
    for (int i = 0; i < num; i++)
    {
        *((float *)p) = (float)(points[i](0));
        p += sizeof(float);
        *((float *)p) = (float)(points[i](1));
        p += sizeof(float);
        *((float *)p) = (float)(intensities[i]);
        p += sizeof(float);
    }
    output->reserve(output->size() + size);
    for (int i = 0; i < size; i++)
    {
        output->push_back(buf[i]);
    }
}
void PointCloudData::from_char_array(char *buf, int size)
{
    if (size >= sizeof(long))
    {
        timestamp = *((long *)buf);
        char *p = buf + sizeof(long);
        size -= sizeof(long);
        points.clear();
        intensities.clear();
        int num = size / (sizeof(float) * 3);
        for (int i = 0; i < num; i++)
        {
            float x = *((float *)p);
            p += sizeof(float);
            float y = *((float *)p);
            p += sizeof(float);
            float in = *((float *)p);
            p += sizeof(float);
            add_point(x, y, in);
        }
    }
}

