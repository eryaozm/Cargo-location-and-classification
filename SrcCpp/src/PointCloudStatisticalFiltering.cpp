
#include "PointCloudStatisticalFiltering.h"
#include <pcl/filters/statistical_outlier_removal.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudStatisticalFiltering (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud)
{
    // 创建统计滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(Cloud);
    sor.setMeanK(50); // 设置用于计算平均距离的邻居点数量
    sor.setStddevMulThresh(1.0); // 设置标准差乘数阈值，距离超出阈值的点将被认为是离群点
    sor.filter(*Cloud); // 执行滤波操作
    return Cloud;
}
