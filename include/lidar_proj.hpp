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
        int direction[9][2] = {{0, 1}, {0, -1}, {-1, 0}, 
                                {1, 0}, {0, 0}, {-1, -1}, 
                                {-1, 1}, {1, -1}, {1, 1}};

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
        for(size_t i = 0; i < pointCloudOut->size(); i++)
        {
            auto pt = pointCloudOut->points[i];
            int u = pt.row; 
            int v = pt.col;

            if(u < 1 || u > H - 1 || v < 1 || v > W - 1)
            {
                normalImg.at<cv::Vec3b>(u, v) == cv::Vec3b(0, 0, 0);
                continue;
            }
            
            const int n = 9;
            Eigen::Matrix<double, n, 3> matA0;
            Eigen::Matrix<double, n, 1> matB0 = -1 * Eigen::Matrix<double, n, 1>::Ones();
            for(int j = 0; j < n; j++)
            {
                int index = IMAGE_WIDTH * (u + direction[j][0]) + v + direction[j][1];
                auto p = pointCloudOut->points[index];
                matA0(j, 0) = p.x;
                matA0(j, 1) = p.y;
                matA0(j, 2) = p.z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            norm.normalize();

            int b = 0, g = 0, r = 0;
            b = (1 + norm.x()) * 127;
            g = (1 + norm.y()) * 127;
            r = (1 + norm.z()) * 127;
            normalImg.at<cv::Vec3b>(u, v)[0] = b;
            normalImg.at<cv::Vec3b>(u, v)[1] = g;
            normalImg.at<cv::Vec3b>(u, v)[2] = r;
        }

        return 1;
    }

private:

    pcl::PointCloud<PointCloudXYZIRCR>::Ptr pointCloudOut;
    cv::Mat rangeImg;
    cv::Mat intensityImg;
    cv::Mat normalImg;
};

