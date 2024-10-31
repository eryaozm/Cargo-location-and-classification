#include  "RemoveGroundFromAllPointClouds.h"
#include "RANSECRemoveGround.h"
#include "Macro.h"
#include <iostream>

void removeGroundFromAllPointClouds() {
    for(int i=0;i<LIDAR_NUM;i++)
    {
        RANSECRemoveGround(LIDAR_PATH_PART +
            std::to_string(i+1) +
            ".ply");
        std::cout << "LIDAR_" << i+1 << "'s ground has been removed" << std::endl;
    }
}
