#pragma once

#include <math.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 

//OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "common.h"

template<typename PointT>
class ProjLidar2Img{
public:
    ProjLidar2Img()
    {
        pointCloudOut.reset(new pcl::PointCloud<PointCloudXYZIRCR>);
    }

    pcl::PointCloud<PointCloudXYZIRCR>::Ptr getPointCloudAfterProcess()
    {
        return pointCloudOut;
    }

    cv::Mat getRangeImg()
    {
        return rangeImg;
    }

    cv::Mat getIntensityImg()
    {
        return intensityImg;
    }

    cv::Mat getNormalImg()
    {
        return normalImg;
    }

    bool rangeProjection(typename pcl::PointCloud<PointT>::Ptr &pc_in,
                        int H = 64, int W = 900, 
                        double fov_up_deg = 3.0, double fov_down_deg = -25.0, 
                        double max_range = 50, double min_range = 0)
    {
        rangeImg = cv::Mat::zeros(H, W, CV_8UC1);
        intensityImg = cv::Mat::zeros(H, W, CV_8UC1);
        normalImg = cv::Mat::zeros(H, W, CV_8UC3);

        double fov_up = fov_up_deg/180 * M_PI;
        double fov_down = fov_down_deg/180 * M_PI;
        double fov = std::abs(fov_down) + std::abs(fov_up);

        pointCloudOut->points.resize(pc_in->size());
        for(size_t i = 0; i < pc_in->size(); i++)
        {
            PointT pt = pc_in->points[i];
            double depth = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if(depth <= min_range || depth > max_range)
                continue;
            if(pt.x == 0 || pt.y == 0 || pt.z == 0)
                continue;
            if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
                continue;
            double scan_x = pt.x;
            double scan_y = pt.y;
            double scan_z = pt.z;
            double intensity = pt.intensity;

            double yaw = -atan2(scan_y, scan_x);
            double pitch = asin(scan_z/depth);

            double proj_x = 0.5 * (yaw/M_PI + 1.0);
            double proj_y = 1 - (pitch + std::abs(fov_down))/fov; 
            
            proj_x *= W;
            proj_y *= H;

            proj_x = std::floor(proj_x);
            if(proj_x > W - 1)
                proj_x = W - 1;
            if(proj_x < 0)
                proj_x = 0;

            proj_y = std::floor(proj_y);
            if(proj_y > H - 1)
                proj_y = H - 1;
            if(proj_y < 0)
                proj_y = 0;

            PointCloudXYZIRCR p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            p.row = proj_y;
            p.col = proj_x;
            p.range = depth;
            pointCloudOut->points[IMAGE_WIDTH * p.row + p.col] = p;
            
            rangeImg.at<uchar>(proj_y, proj_x) = (int)(255 * depth/max_range);
            intensityImg.at<uchar>(proj_y, proj_x) = (int)intensity;
        }
        
        // normal image
        for(size_t i = 0; i < pc_in->size(); i++ )
        {
            if(1)
            {

            }
        }

        return 1;
    }

    bool generateNormalMap()
    {  
        return 1; 
    } 

private:

    pcl::PointCloud<PointCloudXYZIRCR>::Ptr pointCloudOut;
    cv::Mat rangeImg;
    cv::Mat intensityImg;
    cv::Mat normalImg;
};

