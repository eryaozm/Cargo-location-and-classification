#include "AcqirePointCloudFromCoppeliaSim.h"
#include "Macro.h"
#include <vector>
#include <fstream>
#include <string>

void saveToPly(const std::vector<double>& points, const std::string& filename) {
    // 检查点的数量是否为3的倍数
    if (points.size() % 3 != 0) {
        std::cerr << "Error: The vector size is not a multiple of 3." << std::endl;
        return;
    }
    std::ofstream plyFile;
    plyFile.open(filename);
    if (!plyFile) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return;
    }

    // 写入PLY文件头
    plyFile << "ply\n";
    plyFile << "format ascii 1.0\n";
    plyFile << "element vertex " << points.size() / 3 << "\n";
    plyFile << "property float x\n";
    plyFile << "property float y\n";
    plyFile << "property float z\n";
    plyFile << "end_header\n";

    // 写入点数据
    for (size_t i = 0; i < points.size(); i += 3) {
        plyFile << points[i] << " " << points[i + 1] << " " << points[i + 2] << "\n";
    }
    // 关闭文件
    plyFile.close();
}

void acquirePointCloudFromCoppeliaSim(RemoteAPIObject::sim sim)
{
    // 创建向量存储坐标
    std::vector<double>* ptCloudVector[LIDAR_NUM];
    int ptCloudHandle[LIDAR_NUM] = {0};
    for(int i=0;i<LIDAR_NUM;i++) {
        ptCloudVector[i] = new std::vector<double>;
        ptCloudHandle[i] = sim.getObject("/ptCloud"+ std::to_string(i+1));
    }

    // 开始仿真，Lidar开始扫描环境
    sim.setStepping(true);
    sim.startSimulation();
    double t = 0.0;
    while (t < 2.1) {
        t = sim.getSimulationTime();
        printf("Simulation time: %.2f [s]\n", t);
        for(int i=0;i<LIDAR_NUM;i++) {
            std::vector<double> ptCloudTemp = sim.getPointCloudPoints(ptCloudHandle[i]);
            ptCloudVector[i]->insert(ptCloudVector[i]->end(),
                ptCloudTemp.begin(),
                ptCloudTemp.end());
        }
        sim.step();
    }
    // 结束仿真
    sim.stopSimulation();
    //sim.pauseSimulation();
    printf("Simulation paused.\n");
    // 点云坐标写入PLY文件
    for(int i=0;i<LIDAR_NUM;i++) {
        saveToPly(*ptCloudVector[i], LIDAR_PATH_PART +
            std::to_string(i+1)+".ply");
        delete ptCloudVector[i];
    }
    printf("Point cloud data has been written to files.\n");
}