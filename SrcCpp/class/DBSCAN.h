
#ifndef DBSCAN_H
#define DBSCAN_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <set>
#include <Eigen/Dense>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class DBSCAN {
public:
    double eps_;
    int minPts_;
    std::vector<bool> visited_;
    std::vector<std::vector<int>> clusters_;

    DBSCAN(double eps, int minPts);
    void fit(const PointCloudT::Ptr& cloud);
    std::vector<std::vector<int>>& getClusters();
    void expandCluster(const PointCloudT::Ptr& cloud, int idx, const std::vector<int>& neighbors, std::vector<int>& cluster);
    std::vector<int> regionQuery(const PointCloudT::Ptr& cloud, int idx);
    double euclideanDistance(const PointT& p1, const PointT& p2);


};



#endif //DBSCAN_H
