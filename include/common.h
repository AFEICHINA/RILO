#pragma once

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/registration/icp.h>

//opencv
#include <opencv/cv.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

//stl
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <cassert>

#include "ThirdParty/DVision/DVision.h"

typedef pcl::PointXYZI PointType;

extern int IMAGE_WIDTH;
extern int IMAGE_HEIGHT;
extern int IMAGE_CROP;
extern int USE_BRIEF;
extern int USE_ORB;
extern int NUM_BRI_FEATURES;
extern int NUM_ORB_FEATURES;
extern int MIN_LOOP_FEATURE_NUM;
extern double SKIP_TIME;
extern int NUM_THREADS;
extern int DEBUG_IMAGE;
extern double MATCH_IMAGE_SCALE;
extern cv::Mat MASK;

struct PointCloudXYZIRCR{
    PCL_ADD_POINT4D;
    float intensity;
    int row;
    int col;
    double range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointCloudXYZIRCR,
    (float,x,x)
    (float,y,y)
    (float,z,z)
    (float,intensity, intensity)
    (int, row, row)
    (int, col, col)
    (double,range,range)
)

class BriefExtractor
{
public:

    DVision::BRIEF m_brief;

    virtual void operator()(const cv::Mat &im, std::vector<cv::KeyPoint> &keys, std::vector<DVision::BRIEF::bitset> &descriptors) const
    {
        m_brief.compute(im, keys, descriptors);
    }

    BriefExtractor(){};

    BriefExtractor(const std::string &pattern_file)
    {
        cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
        if(!fs.isOpened()) throw std::string("Could not open file ") + pattern_file;

        std::vector<int> x1, y1, x2, y2;
        fs["x1"] >> x1;
        fs["x2"] >> x2;
        fs["y1"] >> y1;
        fs["y2"] >> y2;

        m_brief.importPairs(x1, y1, x2, y2);
    }
};

extern BriefExtractor briefExtractor;
