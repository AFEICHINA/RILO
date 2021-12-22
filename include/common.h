#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace RILO{

struct PointCloudXYZIRCR{
    PCL_ADD_POINT4D;
    float intensity;
    int row;
    int col;
    double range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

}

POINT_CLOUD_REGISTER_POINT_STRUCT(RILO::PointCloudXYZIRCR,
    (float,x,x)
    (float,y,y)
    (float,z,z)
    (float,intensity, intensity)
    (int, row, row)
    (int, col, col)
    (double,range,range)
)
