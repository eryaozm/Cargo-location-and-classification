#include "PointCloudMerge.h"
#include <pcl/filters/voxel_grid.h>
#include "ReadPointCloudFile.h"
#include <fstream>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include "Types.h"
#include "Macro.h"
#include <filesystem>
#include "PointCloudRegistration.h"

MatrixXX readMatrixFromBinary(const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (file.is_open()) {
        int rows, cols;
        file.read(reinterpret_cast<char*>(&rows), sizeof(int));  // 读取行数
        file.read(reinterpret_cast<char*>(&cols), sizeof(int));  // 读取列数
        MatrixXX matrix(rows, cols);
        file.read(reinterpret_cast<char*>(matrix.data()), rows * cols * sizeof(double));  // 读取矩阵数据
        file.close();
        return matrix;
    } else {
        std::cerr << "Unable to open file for reading!" << std::endl;
        return MatrixXX();
    }
}

void mergeAllPointCloud()
{
    Eigen::Matrix<double, 4, 4> transformation_matrix[LIDAR_NUM-1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud[LIDAR_NUM];
    for(int i=0;i<LIDAR_NUM;i++)
        lidar_cloud[i] = readPointCloudFile(LIDAR_PATH_PART + std::to_string(i+1) + ".ply");
    pcl::PLYWriter writer;
    std::filesystem::path MatrixDir = MATRIX_DIR;
    std::cout << "Layer 1 reconstruction:" << std::endl;
    // 一层还原
    for(int i = 0; i < LIDAR_NUM-1; i++) {
        std::string file_name = std::string(MATRIX_DIR) + "/" + std::to_string(1) + "_" + std::to_string(i) + ".bin";
        transformation_matrix[i] = readMatrixFromBinary(file_name);
        std::cout << "Current Transform Matrix:" << std::endl << transformation_matrix[i] << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*lidar_cloud[i+1],*temp,
            transformation_matrix[i].cast<float>());
        *(lidar_cloud[i]) += *temp;
        downSample(lidar_cloud[i]);
    }

    // 高层还原
    for(int j=2;j<LIDAR_NUM;j++) {
        std::cout << "Layer "+ std::to_string(j) + " reconstruction:" << std::endl;
        for(int i=0; i < LIDAR_NUM-j; i++) {
            std::pair temp_pair = pcPtr2File(lidar_cloud[i],lidar_cloud[i+1]);
            std::string file_name = std::string(MATRIX_DIR) + "/" + std::to_string(j) + "_" + std::to_string(i) + ".bin";
            transformation_matrix[i] = readMatrixFromBinary(file_name);
            std::cout << "Current Transform Matrix:" << std::endl << transformation_matrix[i] << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*lidar_cloud[i+1],*temp,
                transformation_matrix[i].cast<float>());
            *(lidar_cloud[i]) += *temp;
            downSample(lidar_cloud[i]);
        }
    }
    writer.write(ALL_LIDAR_FILEPATH,*(lidar_cloud[0]),false);
}