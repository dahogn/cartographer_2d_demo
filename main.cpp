#include <iostream>
#include <fstream>

#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>

#include "carto.h"
#include "sensor_data.h"
#include "pose.h"
#include "simple_grid_map.h"

using namespace std;

Position last_pose;
/**
 * Handle pose updating from cartographer.
 * Cartographer will expolate and optimize pose for each radar pointcloud,
 *  which can be treat as realtime pose of robot during mapping.
 * But, it might be incorrect after final optimization.
 */
/**
 * 用于处理cartographer返回的位姿估计的函数
 * Cartographer会为每一帧激光雷达数据估计一个位姿并通过此函数返回
 * 该位姿可以作为建图过程中机器人的位姿
 * 但最终优化之后可能不准确（因为优化过程中地图变化了）
 */
void pose_change_callback(Position &pose) {
    if(pose.timestamp > last_pose.timestamp) {
        last_pose = pose;
    }
}

int main(int argc, char *argv[]) {

    if(argc < 5) {
        cout << "Usage: " << argv[0] << " config_dir config_file replay_bag map_output_path" << endl;
        return 1;
    }

    //Initialize cartographer module
    //初始化
    CartoModule *carto = new CartoModule(argv[1], argv[2], pose_change_callback);

    //Decode senser datas from replaybag file
    //读取回放文件中的传感器数据并输入
    ifstream ifile(argv[3], ios::in | ios::binary);
    int data_size = 0, type = 0;
    char *buf;
    while(!ifile.eof()) {
        ifile.read((char*)(&data_size), sizeof(int));
        if(data_size <= 0) {
            cerr << "Replay data format error" << endl;
            return 1;
        }
        ifile.read((char*)(&type), sizeof(int));
        buf = new char[data_size];
        ifile.read(buf, data_size);
        if(type == 1) {
            ImuData2D imu_data;
            imu_data.from_char_array(buf, data_size);
            carto->handle_imu_data(imu_data);
        }
        else if(type == 2) {
            PointCloudData radar_data;
            radar_data.from_char_array(buf, data_size);
            carto->handle_radar_data(radar_data);
        }
    }
    //Stop cartographer
    //停止建图
    carto->stop_and_optimize();
    
    //Get map
    //获取地图数据
    vector<char> map_data;
    carto->paint_map(&map_data);

    delete carto;

    //Example of class SimpleGridMap:
    //地图对象的使用示例:
    SimpleGridMap *map = new SimpleGridMap(map_data.data(), map_data.size());
    MapInfo info = map->get_info();
    //Grid numbers in this grid map:
    //地图中的格栅总数
    int grid_size = info.width * info.height;
    //Map range in global coordinate system, where (0,0) means the start position
    //全局坐标系中的地图范围，坐标系原点为建图的起始位置
    double min_x = info.origen_x, max_x = info.origen_x + info.resolution * info.width;
    double min_y = info.origen_y, max_y = info.origen_y + info.resolution * info.height;
    //Get grid value of point (x,y) in global coordinate system, return -1 as Unobserved, 0-1 as occupied possibility
    //获取全局坐标系中的点(x,y)的格栅值，-1表示未观测到，0-1表示格栅占据概率
    double value = map->get(0.5, 1.0);
    if(value < 0) {
        //Unobserved
    }
    else {
        if(value > 0.4) {
            //Occupied
        }
        else {
            //Free
        }
    }

    delete map;

    cout << "Output map to file " << argv[4] << endl;
    ofstream ofile(argv[4], ios::out | ios::binary);
    ofile.write(map_data.data(), map_data.size());
    ofile.close();

    return 0;
}


