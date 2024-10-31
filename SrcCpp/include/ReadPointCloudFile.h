
#ifndef READPOINTCLOUDFILE_H
#define READPOINTCLOUDFILE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr readPointCloudFile(std::string filename);

#endif //READPOINTCLOUDFILE_H
