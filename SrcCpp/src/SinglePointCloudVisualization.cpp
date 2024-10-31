#include "SinglePointCloudVisualization.h"
#include <pcl/visualization/pcl_visualizer.h>
#include "ReadPointCloudFile.h"
#include "Macro.h"
int singlePointCLoudVisualization() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readPointCloudFile(ALL_LIDAR_FILEPATH);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色
    // 按照Z轴值对点云进行上色
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, "single cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "single cloud"); // 设置点的大小
    viewer->addCoordinateSystem(1.0); // 添加坐标轴
    viewer->initCameraParameters(); // 初始化摄像头参数

    while (!viewer->wasStopped())
    {
        viewer->spin(); // 使可视化窗口不停刷新
    }

    return 0;
}
