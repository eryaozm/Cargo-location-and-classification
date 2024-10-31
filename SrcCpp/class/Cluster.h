#ifndef CLUSTER_H
#define CLUSTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

class Cluster {
public:
    int id;
    int type_id;
    int num;
    std::string file_name;
    std::string type_name;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointXYZ centroid;
    pcl::PointXYZ highest_point;
    Cluster();
    ~Cluster();
    void coordinateTransformation();
    void centroidCompute();
    void highestPointCompute();

};

#endif //CLUSTER_H
