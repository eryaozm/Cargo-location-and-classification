
#ifndef MACRO_H
#define MACRO_H

#define LIDAR_NUM 8
#define MATRIX_DIR "RegistritionConfig"
#define LIDAR_PATH_PART "C:/Users/eryao/Documents/CLionProjects/PointCloud/data/lidar"
#define ALL_LIDAR_FILEPATH "C:/Users/eryao/Documents/CLionProjects/PointCloud/data/lidar_all.ply"
#define POINTNET "C:/Users/eryao/miniconda3/envs/torch/python.exe C:/Users/eryao/Documents/PycharmProjects/PointClassification/test.py"
// 钩子初始位置在世界坐标系中的坐标
#define HOOK_ZERO_X (-9.5300326630217497)
#define HOOK_ZERO_Y (-9.5990418640692585)
#define HOOK_ZERO_Z 8.0200896687106322
// 点云原点在世界坐标系中的坐标
#define RELATIVE_X 30.0
#define RELATIVE_Y (-30.0)
#define RELATIVE_Z 5.0
// 点云坐标转换到世界坐标
#define T_X(a) (a+RELATIVE_X)
#define T_Y(a) (a+RELATIVE_Y)
#define T_Z(a) (a+RELATIVE_Z)

// 世界坐标系转换到点云坐标系
#define TP_X(a) (a-RELATIVE_X)
#define TP_Y(a) (a-RELATIVE_Y)
#define TP_Z(a) (a-RELATIVE_Z)

// 起重机各关节的速度
#define VelX 3.0
#define VelY 3.0
#define VelZ 3.0

#endif //MACRO_H
