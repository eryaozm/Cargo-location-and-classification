#include "PointCloudClassification.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include "Macro.h"

void pointCloudClassification(Cluster* cluster_array)
{
    // 读取类别
    std::ifstream file("C:/Users/eryao/Documents/PycharmProjects/PointClassification/data/shape_names.txt");
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
        lines.push_back(line);
    }
    file.close();
    // 运行网络对点云分类
    int ret = system(POINTNET);
    if (ret == 0) {
        std::cout << "Clusters have completed classification.\n\n";
    } else {
        std::cout << "Classification error!\n" << std::endl;
        exit(-1);
    }
    // 读取结果
    std::ifstream result_file("C:/Users/eryao/Documents/PycharmProjects/PointClassification/actual_data/result.txt");
    std::vector<int> result;
    std::string result_line;
    while (std::getline(result_file, result_line)) {
        result.push_back(std::stoi(result_line));
    }
    result_file.close();
    for(int i = 0; i < cluster_array[0].num; i++)
    {
        cluster_array[i].type_id = result[i];
        cluster_array[i].type_name = lines[cluster_array[i].type_id];
        cluster_array[i].centroidCompute(); //计算中心点
        cluster_array[i].highestPointCompute(); //计算最高点
        cluster_array[i].coordinateTransformation(); //转换到世界坐标系
        std::cout << "id:" <<cluster_array[i].id << '\n' <<
            "file_name:" << cluster_array[i].file_name << '\n' <<
            "type_id:" << cluster_array[i].type_id << '\n' <<
            "type_name:" << cluster_array[i].type_name << '\n' <<
            "centroid:" << cluster_array[i].centroid << '\n' <<
            "highest_point:" << cluster_array[i].highest_point << "\n" <<std::endl;
    }
}
