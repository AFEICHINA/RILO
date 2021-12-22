#include "iostream"
#include "data_reader.hpp"
#include "proj_lidar.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

typedef pcl::PointXYZI PointType;

using namespace RILO;

int main(int argc, char* argv[])
{
    RILO::DataReader<PointType> data;
    std::string fileName = "/home/shl/syf/RILO/data/pcd/000000.pcd";
    pcl::PointCloud<PointType>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<PointType>);
    data.readCloudFile(fileName, cloud);

    cv::Mat range_img;
    cv::Mat intensity_img;
    ProjLidar2Img<PointType> proj_lidar;
    proj_lidar.rangeProjection(cloud);
    range_img = proj_lidar.getRangeImg();
    intensity_img = proj_lidar.getIntensityImg();
    cv::imshow("range img", range_img);
    cv::imshow("intensity img", intensity_img);
    cv::waitKey(0);


    return 0;
}