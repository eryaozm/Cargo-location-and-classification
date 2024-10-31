#include "DBSCAN.h"

DBSCAN::DBSCAN(double eps, int minPts) : eps_(eps), minPts_(minPts) {}

void DBSCAN::fit(const PointCloudT::Ptr& cloud) {
    clusters_.clear();
    visited_.assign(cloud->size(), false);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!visited_[i]) {
            visited_[i] = true;
            std::vector<int> neighbors = regionQuery(cloud, i);
            if (neighbors.size() >= minPts_) {
                std::vector<int> cluster;
                expandCluster(cloud, i, neighbors, cluster);
                clusters_.push_back(cluster);
            }
        }
    }
}

void DBSCAN::expandCluster(const PointCloudT::Ptr& cloud, int idx, const std::vector<int>& neighbors, std::vector<int>& cluster) {
    cluster.push_back(idx);
    std::set<int> seeds(neighbors.begin(), neighbors.end());
    while (!seeds.empty()) {
        int current = *seeds.begin();
        seeds.erase(seeds.begin());

        if (!visited_[current]) {
            visited_[current] = true;
            std::vector<int> newNeighbors = regionQuery(cloud, current);
            if (newNeighbors.size() >= minPts_) {
                seeds.insert(newNeighbors.begin(), newNeighbors.end());
            }
        }

        if (std::find(cluster.begin(), cluster.end(), current) == cluster.end()) {
            cluster.push_back(current);
        }
    }
}

std::vector<int> DBSCAN::regionQuery(const PointCloudT::Ptr& cloud, int idx) {
    std::vector<int> neighbors;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (euclideanDistance(cloud->points[idx], cloud->points[i]) <= eps_) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

double DBSCAN::euclideanDistance(const PointT& p1, const PointT& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

std::vector<std::vector<int>>& DBSCAN::getClusters()
{
    return clusters_;
}
