#include <iostream>
#include <string>
#include <cmath>
#include <pcl/console/print.h> //停用警告信息
#include "Macro.h" //定义的宏
#include "RemoteAPIClient.h" //CoppeliaSim远程控制
#include "AcqirePointCloudFromCoppeliaSim.h" //从仿真环境获取点云
#include "RemoveGroundFromAllPointClouds.h"
#include "ReadPointCloudFile.h" //点云读取
#include "PointCloudStatisticalFiltering.h" //点云统计滤波
#include "PointCloudRegistration.h" //点云RICP配准
#include "SinglePointCloudVisualization.h" //单一点云的可视化
#include "PointCloudMerge.h" //根绝已有配准信息合并点云
#include "PointCloudCluster.h" //点云聚类
#include "PointCloudClassification.h" //调用网络为聚类分类
#include "Cluster.h"
#include "Dispatch.h" //调度起重机运动
#include "RemoveCrane.h" //从点云中移除起重机对应的部分

int main(int argc,char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    // 初始化远程连接
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    // 采集激光雷达点云
    acquirePointCloudFromCoppeliaSim(sim);

    // 去除地面
    removeGroundFromAllPointClouds();

    // 点云配准
    // registrationAllPointCloud();

    // 利用配准信息重构场景
    mergeAllPointCloud();

    // 去除起重机
    removeCrane();

    // // 查看配准效果
    singlePointCLoudVisualization();

    // 点云聚类
    Cluster* cluster_array = pointCloudCluster();

    // 利用网络分类点云并对聚类后处理
    pointCloudClassification(cluster_array);

    // 设置目标位置
    auto* position = new Position();
    position->x = atof(argv[1]), position->y = atof(argv[2]);

    // 设置操作标志
    bool* act = new bool;
    *act = true;

    // 分配调度任务
    sim.setStepping(true);
    sim.startSimulation();
    Dispatch(cluster_array,act,position,argv[3],sim);
    return 0;
}
