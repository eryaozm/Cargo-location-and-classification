#include "PointCloudCluster.h"
#include "Macro.h"
#include "ReadPointCloudFile.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include "DBSCAN.h"
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <fstream>

Cluster* pointCloudCluster()
{
    std::cout << "Start point cloud clustering." << std::endl;
    // 读取场景
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = readPointCloudFile(ALL_LIDAR_FILEPATH);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(scene);

    // 设置欧几里得聚类的参数
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5); // 设置聚类的距离阈值（0.5m）
    ec.setMinClusterSize(300);    // 聚类中最少点数
    ec.setMaxClusterSize(25000);  // 聚类中最多点数
    ec.setSearchMethod(tree);     // 设置搜索方式为KdTree
    ec.setInputCloud(scene);      // 输入点云
    ec.extract(cluster_indices);  // 提取聚类结果

    int size = cluster_indices.size();
    std::cout << "The size of data is " << size << std::endl;
    auto* Cluster_array = new Cluster[size];
    int cluster_id = 0;
    //  提取每个聚类并分别保存
    for (const auto& indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : indices.indices)
            cloud_cluster->points.push_back(scene->points[idx]); // 添加属于同一个聚类的点
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // 聚类保存到txt文件
        std::string cluster_name;
        cluster_name.append("C:/Users/eryao/Documents/PycharmProjects/PointClassification/actual_data/data/clusster_");
        cluster_name.append(std::to_string(cluster_id));
        cluster_name.append(".txt");
        std::ofstream outFile(cluster_name);
        for (const auto& point : cloud_cluster->points)
        {
            outFile << point.x << " " << point.y << " " << point.z << std::endl;
        }
        outFile.close();

        Cluster_array[cluster_id].id = cluster_id; //拷贝id
        *Cluster_array[cluster_id].cloud = *cloud_cluster; //拷贝点云
        Cluster_array[cluster_id].file_name = cluster_name; //拷贝文件名
        Cluster_array[cluster_id].num = size;
        cluster_id++;
    }
    std::cout << "Finish point cloud clustering." << std::endl;
    return Cluster_array;
}