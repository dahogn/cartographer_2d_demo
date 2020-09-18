#include "simple_grid_map.h"

#include <stdio.h>

SimpleGridMap::SimpleGridMap(MapInfo &info) : map_info(info)
{
    pthread_mutex_init(&mutex, nullptr);
};

SimpleGridMap::SimpleGridMap(SimpleGridMap &map)
{
    pthread_mutex_init(&mutex, nullptr);
    map_info = map.get_info();
    datas = map.datas;
}

SimpleGridMap::SimpleGridMap(char *data_buf, int buf_size)
{
    pthread_mutex_init(&mutex, nullptr);
    if(buf_size == 0) {
        return;
    }
    from_char_array(data_buf, buf_size);
};

SimpleGridMap::~SimpleGridMap()
{
    pthread_mutex_destroy(&mutex);
};

void SimpleGridMap::to_char_array(std::vector<char> *output) {
    pthread_mutex_lock(&mutex);
    int data_size = datas.size();
    int size = data_size + sizeof(MapInfo);
    output->reserve(output->size() + size);
    char *p_info = (char *)(&map_info);
    for (int i = 0; i < sizeof(MapInfo); i++)
    {
        output->push_back(p_info[i]);
    }
    for (int i = 0; i < data_size; i++)
    {
        output->push_back(datas[i]);
    }
    pthread_mutex_unlock(&mutex);
}

void SimpleGridMap::from_char_array(char *buf, int size) {
    if (size < sizeof(MapInfo))
    {
        printf("Decompress map data error : No map data\n");
        return;
    }
    pthread_mutex_lock(&mutex);
    MapInfo info = *((MapInfo *)(buf));
    map_info = info;
    datas.reserve(info.width * info.height);
    for (int i = sizeof(MapInfo); i < size; i++)
    {
        datas.push_back(buf[i]);
    }
    printf("New map infomation: %dx%d, origin (%.3f, %.3f), resolution %f \n", info.width, info.height, info.origen_x, info.origen_y, info.resolution);
    pthread_mutex_unlock(&mutex);
}

double SimpleGridMap::get(double x, double y)
{
    pthread_mutex_lock(&mutex);
    int x_map = map_info.global_x_to_map_x(x);
    int y_map = map_info.global_y_to_map_y(y);
    if (x_map < 0 || y_map < 0 || x_map >= map_info.width || y_map >= map_info.height)
    {
        pthread_mutex_unlock(&mutex);
        return -1;
    }
    int data = (datas[map_info.map_xy_to_array_index(x_map, y_map)] & 0xff);
    pthread_mutex_unlock(&mutex);
    if (data == 0xff)
    {
        return -1;
    }
    return ((double)data / 100.);
}
