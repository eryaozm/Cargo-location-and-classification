#include "RANSECRemoveGround.h"
#include "ReadPointCloudFile.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/ply_io.h>
#include "PointCloudStatisticalFiltering.h"
#include <pcl/filters/voxel_grid.h>

void RANSECRemoveGround (std::string FileName)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readPointCloudFile(FileName);
    // 创建平面分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_ground(new pcl::PointCloud<pcl::PointXYZ>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1); // 设置地面点与模型的距离阈值，根据需要调整
    // 执行平面分割
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return;
    }
    // 提取非地面部分的点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // 设置为 true 以提取非地面部分
    extract.filter(*cloud_without_ground);
    pcl::PLYWriter writer;
    writer.write(FileName, *cloud_without_ground, false);
}

