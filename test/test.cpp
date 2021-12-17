#include "iostream"
#include "data_reader.hpp"

typedef pcl::PointXYZI PointType;
int main(int argc, char* argv[])
{
    RILO::DataReader<PointType> data;
    std::string fileName = "/home/shl/syf/RILO/data/pcd/000000.pcd";
    pcl::PointCloud<PointType>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<PointType>);
    data.readCloudFile(fileName, cloud);

    
    return 0;
}