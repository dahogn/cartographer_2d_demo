# cartographer_2d_demo
Use google cartographer 2D SLAM without ROS. <br>
在非ROS环境下使用cartographer进行2D激光SLAM的测试。

---

Replay files containing imu and radar datas will be uploaded soon...<br>
之后会上传包含传感器数据的回放文件...

---
# Build
## Install cartographer libary
Compile and install `abseil`, `ceres-solver`, `proto-buf` and `cartographer` followed the document below:<br>
根据cartographer官方文档编译安装Cartographer及其依赖库：<br>
[Cartographer Document](https://google-cartographer.readthedocs.io/en/latest/)
## Build 
Enter directory `cartographer_2d_demo`<br>
```Bash
mkdir build
cd build
cmake ..
make
```
# Run
After building, excutable file `map_test` will be generated.<br>
Run this project by:
```Bash
./map_test ../configuration_files mapping_imu.lua ../data/imu_radar_replay ./map_output
```

