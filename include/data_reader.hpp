#pragma once

#include <chrono>

//PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include "point_cloud.hpp"

namespace RILO{

template<typename PointT>
class DataReader{
public:
    bool checkDir(const std::string &dir)
    {
        if(boost::filesystem::exists(dir.c_str()))
        {
            if(boost::filesystem::create_directory(dir.c_str()))
                return true;
            else
                return false;
        }
        return true;
    }

    inline bool exitsFile(const std::string &filename)
    {
        std::ifstream f(filename.c_str());
        return f.good();
    }

    bool readCloudFile(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

        std::string suffix = fileName.substr(fileName.find_last_of(".") + 1);

        if(!strcmp(suffix.c_str(), "pcd"))
        {
            readPCDFile(fileName, pointCloud);
        }
        else if(!strcmp(suffix.c_str(), "txt"))
        {
            readTXTFile(fileName, pointCloud);
        }
        else if(!strcmp(suffix.c_str(), "bin"))
        {
            readBinFile(fileName, pointCloud);
        }
        else
        {
            return 0;
        }
        std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used =  std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

        std::cout << "[" << pointCloud->size() << "] points loaded in [" << time_used.count() * 1000 << "] ms" << std::endl;
        return 1;
    }

    bool readPCDFile(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't read file\n");
            return false;
        }
        return true;
    }

    bool readTXTFile(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::ifstream in(fileName.c_str(), std::ios::in);
        if (!in)
        {
            return 0;
        }
        double x_ = 0, y_ = 0, z_ = 0, intensity_ = 0;
        int i = 0;
        while (!in.eof())
        {
            in >> x_ >> y_ >> z_ >> intensity_;
            if (in.fail())
            {
                break;
            }
            PointT Pt;
            Pt.x = x_;
            Pt.y = y_;
            Pt.z = z_;
            Pt.intensity = intensity_;
            pointCloud->points.push_back(Pt);
            ++i;
        }
        in.close();
        //std::cout << "Import finished ... ..." << std::endl;
        return 1;
    }

    bool readBinFile(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        // Load point cloud stored as KITTI's bin format
	    std::fstream input(fileName.c_str(), std::ios::in | std::ios::binary);
	    if(!input.good()){
		    // LOG(ERROR) << "Could not read *.bin file: " << file_name;
		    return false;
	    }
	    input.seekg(0, std::ios::beg);
 
	    int i;
	    for (i=0; input.good() && !input.eof(); i++) {
		    PointT point;
		    input.read((char *) &point.x, 3*sizeof(float));
		    input.read((char *) &point.intensity, sizeof(float));
            point.intensity*=255;
		    pointCloud->push_back(point);
	    }
	    input.close();
        return true;
    }

};

} // RILO