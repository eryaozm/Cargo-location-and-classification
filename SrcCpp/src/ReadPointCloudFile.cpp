#include "ReadPointCloudFile.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>

pcl::PointCloud<pcl::PointXYZ>::Ptr readPointCloudFile(std::string FileName) {

    //std::string extension = boost::filesystem::extension(FileName);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 根据文件扩展名选择合适的读取方式
    if (std::filesystem::path(FileName).extension() == ".pcd")
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(FileName, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read PCD file %s\n", FileName.c_str());
            return nullptr;
        }
    }
    else if (std::filesystem::path(FileName).extension() == ".ply")
    {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(FileName, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read PLY file %s\n", FileName.c_str());
            return nullptr;
        }
    }
    else
    {
        std::cerr << "Unsupported file format: " << std::filesystem::path(FileName).extension() << std::endl;
        return nullptr;
    }
    return cloud;
}
