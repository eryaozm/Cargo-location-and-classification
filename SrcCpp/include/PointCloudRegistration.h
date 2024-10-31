
#ifndef POINTCLOUDREGISTRATION_H
#define POINTCLOUDREGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
std::pair<std::string, std::string> pcPtr2File(pcl::PointCloud<pcl::PointXYZ>::Ptr a,pcl::PointCloud<pcl::PointXYZ>::Ptr b);
void registrationAllPointCloud();

#endif //POINTCLOUDREGISTRATION_H
