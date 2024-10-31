#include "RemoveCrane.h"
#include "Macro.h"
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include "ReadPointCloudFile.h"

void removeCrane()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr OriginalCloud = readPointCloudFile(ALL_LIDAR_FILEPATH);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(OriginalCloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(TP_X(9.6), TP_X(10.4));
    pass.setNegative(true);
    pass.filter(*OriginalCloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(TP_X(-10.4), TP_X(-9.6));
    pass.setNegative(true);
    pass.filter(*OriginalCloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(TP_Y(-10.4), TP_Y(-9.6));
    pass.setNegative(true);
    pass.filter(*OriginalCloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(TP_Y(9.6), TP_Y(10.4));
    pass.setNegative(true);
    pass.filter(*OriginalCloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(TP_Z(7.5), TP_Z(10));
    pass.setNegative(true);
    pass.filter(*OriginalCloud);


    // OriginalCloud = pointCloudStatisticalFiltering(OriginalCloud);

    pcl::PLYWriter writer;
    writer.write(ALL_LIDAR_FILEPATH, *OriginalCloud);
    std::cout << "Crane has been removed." << std::endl;
}
