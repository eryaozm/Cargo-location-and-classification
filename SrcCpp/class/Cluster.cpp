#include "Cluster.h"
#include "Macro.h"
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <cmath>
#include <iostream>

Cluster::Cluster():cloud(new pcl::PointCloud<pcl::PointXYZ>){}

Cluster::~Cluster(){}

void view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // 绿色
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);  // 每100ms刷新一次
    }
}

// 计算两个点之间的欧氏距离
double calculateDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

// 调整点的距离，使用线性插值的方法移动点
pcl::PointXYZ interpolatePoint(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, double t) {
    pcl::PointXYZ interpolated_point;
    interpolated_point.x = p1.x + t * (p2.x - p1.x);
    interpolated_point.y = p1.y + t * (p2.y - p1.y);
    interpolated_point.z = p1.z + t * (p2.z - p1.z);
    return interpolated_point;
}


// 寻找中心点
void Cluster::centroidCompute()
{
    // 由于激光雷达点云近密远疏的特性，需要通过投影和拟合得到点云的外缘轮廓从而获得中心点
    // 创建一个用于投影的平面Z=0
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);  // ax + by + cz + d = 0
    coefficients->values[0] = 0.0;
    coefficients->values[1] = 0.0;
    coefficients->values[2] = 1.0;  // Z方向法向量
    coefficients->values[3] = 0.0;  // Z=0 平面
    // 将点云投影到 Z=0 平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);  // 选择平面模型
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    // 拟合凸包以提取轮廓
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_projected);
    hull.reconstruct(*hull_cloud);
    // view(hull_cloud);
    // 设置目标点间距
    double target_distance = 0.1;

    // 创建一个新的点云来存储调整后的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr adjusted_hull(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < hull_cloud->points.size(); ++i) {
        size_t next = (i + 1) % hull_cloud->points.size(); // 闭合凸包
        pcl::PointXYZ p1 = hull_cloud->points[i];
        pcl::PointXYZ p2 = hull_cloud->points[next];

        // 添加起始点
        adjusted_hull->points.push_back(p1);

        // 计算起始点和终点的距离
        double current_distance = calculateDistance(p1, p2);

        // 计算需要插入的点的数量
        int num_points_to_insert = static_cast<int>(current_distance / target_distance);

        // 插入点
        for (int j = 1; j < num_points_to_insert; ++j) {
            double t = static_cast<double>(j) / num_points_to_insert;
            pcl::PointXYZ interpolated_point = interpolatePoint(p1, p2, t);
            adjusted_hull->points.push_back(interpolated_point);
        }
    }

    // view(adjusted_hull);

    // 计算拟合出的轮廓XY中心点
    centroid.x = 0.0;
    centroid.y = 0.0;
    for (const auto& point : adjusted_hull->points) {
        centroid.x += point.x;
        centroid.y += point.y;
    }
    if (!adjusted_hull->points.empty()) {
        centroid.x /= adjusted_hull->points.size();
        centroid.y /= adjusted_hull->points.size();
    }

    // 计算原始点云的Z中心点
    centroid.z = 0.0;
    for (const auto& point : cloud->points) {
        centroid.z += point.z;
    }
    if (!cloud->points.empty()) {
        centroid.z /= cloud->points.size();
    }
}

void Cluster::highestPointCompute()
{
    highest_point = cloud->points[0]; // 假设第一个点为 z 最大的点
    // 遍历所有点，找到 z 轴坐标最大的点
    for (const auto& point : cloud->points) {
        if (point.z > highest_point.z) {
            highest_point = point;
        }
    }
}

void Cluster::coordinateTransformation()
{
    // 转换Cluster中心点
    centroid.x = T_X(centroid.x);
    centroid.y = T_Y(centroid.y);
    centroid.z = T_Z(centroid.z);
    // 转换Cluster的最高点
    highest_point.x = T_X(highest_point.x);
    highest_point.y = T_Y(highest_point.y);
    highest_point.z = T_Z(highest_point.z);
}


