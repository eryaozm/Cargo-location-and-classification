#include <filesystem>
#include "PointCloudRegistration.h"
#include "ReadPointCloudFile.h"
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include "ICP.h"
#include "io_pc.h"
#include "FRICP.h"
#include "Macro.h"
#include <pcl/filters/voxel_grid.h>

MatrixXX icp(std::string file_source, std::string file_target)
{
    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;
    typedef Eigen::Matrix<Scalar, 3, 1> VectorN;
    MatrixXX res_trans;
    //--- Model that will be rigidly transformed
    Vertices vertices_source, normal_source, src_vert_colors;
    read_file(vertices_source, normal_source, src_vert_colors, file_source);
    std::cout << "source: " << vertices_source.rows() << "x" << vertices_source.cols() << std::endl;
    //--- Model that source will be aligned to
    Vertices vertices_target, normal_target, tar_vert_colors;
    read_file(vertices_target, normal_target, tar_vert_colors, file_target);
    std::cout << "target: " << vertices_target.rows() << "x" << vertices_target.cols() << std::endl;
    // scaling
    Eigen::Vector3d source_scale, target_scale;
    source_scale = vertices_source.rowwise().maxCoeff() - vertices_source.rowwise().minCoeff();
    target_scale = vertices_target.rowwise().maxCoeff() - vertices_target.rowwise().minCoeff();
    double scale = std::max(source_scale.norm(), target_scale.norm());
    std::cout << "scale = " << scale << std::endl;
    vertices_source /= scale;
    vertices_target /= scale;
    /// De-mean
    VectorN source_mean, target_mean;
    source_mean = vertices_source.rowwise().sum() / double(vertices_source.cols());
    target_mean = vertices_target.rowwise().sum() / double(vertices_target.cols());
    vertices_source.colwise() -= source_mean;
    vertices_target.colwise() -= target_mean;
    // set ICP parameters
    ICP::Parameters pars;
    // set Sparse-ICP parameters
    SICP::Parameters spars;
    spars.p = 10;
    spars.print_icpn = false;
    ///--- Execute registration
    FRICP<3> fricp;
    pars.f = ICP::WELSCH;
    pars.use_AA = true;
    fricp.point_to_point(vertices_source, vertices_target, source_mean, target_mean, pars);
    res_trans = pars.res_trans;
    Eigen::Affine3d res_T;
    res_T.linear() = res_trans.block( 0,0,3,3);  //旋转变换矩阵
    res_T.translation() = res_trans.block(0, 3,3,1);  //平移向量
    res_trans.block(0,3,3,1) *= scale;
    std::cout << "Current Transform Matrix:" << std::endl << res_trans << std::endl;
    return res_trans;
}

// 降采样
void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);  // 设置输入点云
    voxel_filter.setLeafSize(0.02f, 0.02f, 0.02f);  // 设置每个体素的边长为 2cm
    voxel_filter.filter(*cloud);
}

// 转化为完整路径
std::pair<std::string, std::string> pcPtr2File(pcl::PointCloud<pcl::PointXYZ>::Ptr a,pcl::PointCloud<pcl::PointXYZ>::Ptr b) {
    pcl::PLYWriter writer;
    writer.write("a.ply",*a,false);
    writer.write("b.ply",*b,false);
    return std::make_pair("a.ply","b.ply");
}

void saveMatrixToBinary(const MatrixXX& matrix, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (file.is_open()) {
        int rows = matrix.rows();
        int cols = matrix.cols();
        file.write(reinterpret_cast<char*>(&rows), sizeof(int));  // 写入行数
        file.write(reinterpret_cast<char*>(&cols), sizeof(int));  // 写入列数
        file.write(reinterpret_cast<const char*>(matrix.data()), rows * cols * sizeof(double));  // 写入矩阵数据
        file.close();
    } else {
        std::cerr << "Unable to open file for writing!" << std::endl;
    }
}

void registrationAllPointCloud() {
    printf("Start Registration\n");
    Eigen::Matrix<double, 4, 4> transformation_matrix[LIDAR_NUM-1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud[LIDAR_NUM];
    for(int i=0;i<LIDAR_NUM;i++)
        lidar_cloud[i] = readPointCloudFile(LIDAR_PATH_PART + std::to_string(i+1) + ".ply");
    pcl::PLYWriter writer;
    // 创建配准信息配置文件夹
    std::filesystem::path MatrixDir = MATRIX_DIR;
    std::filesystem::create_directory(MatrixDir);

    std::cout << "Layer 1 registration:" << std::endl;
    // 一层配准
    for(int i = 0; i < LIDAR_NUM-1; i++) {
        transformation_matrix[i] = icp(LIDAR_PATH_PART + std::to_string(i+2) + ".ply",
            LIDAR_PATH_PART + std::to_string(i+1) + ".ply");
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*lidar_cloud[i+1],*temp,
            transformation_matrix[i].cast<float>());
        std::string file_name = std::string(MATRIX_DIR) + "/" + std::to_string(1) + "_" + std::to_string(i) + ".bin";
        saveMatrixToBinary(transformation_matrix[i], file_name);
        *(lidar_cloud[i]) += *temp;
        downSample(lidar_cloud[i]);
    }
    // 高层配准
    for(int j=2;j<LIDAR_NUM;j++) {
        std::cout << "Layer "+ std::to_string(j) + " registration:" << std::endl;
        for(int i=0; i < LIDAR_NUM-j; i++) {
            std::pair temp_pair = pcPtr2File(lidar_cloud[i],lidar_cloud[i+1]);
            transformation_matrix[i] = icp(temp_pair.second,temp_pair.first);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*lidar_cloud[i+1],*temp,
                transformation_matrix[i].cast<float>());
            std::string file_name = std::string(MATRIX_DIR) + "/" + std::to_string(j) + "_" + std::to_string(i) + ".bin";
            saveMatrixToBinary(transformation_matrix[i], file_name);
            *(lidar_cloud[i]) += *temp;
            downSample(lidar_cloud[i]);
        }
    }
    writer.write(ALL_LIDAR_FILEPATH,*(lidar_cloud[0]),false);
}
