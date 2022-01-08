#include "iostream"
#include "data_reader.hpp"
#include "lidar_proj.hpp"
#include "keyframe.hpp"
#include "common.h"

#include <boost/format.hpp>

int IMAGE_WIDTH;
int IMAGE_HEIGHT;
int IMAGE_CROP;
int USE_BRIEF;
int USE_ORB;
int NUM_BRI_FEATURES;
int NUM_ORB_FEATURES;
int MIN_LOOP_FEATURE_NUM;
int MIN_LOOP_SEARCH_GAP;
double MIN_LOOP_SEARCH_TIME;
float MIN_LOOP_BOW_TH;
double SKIP_TIME = 0;
int NUM_THREADS;
int DEBUG_IMAGE;
double MATCH_IMAGE_SCALE;
cv::Mat MASK;
BriefExtractor briefExtractor;

int main(int argc, char* argv[])
{
    std::string config_file;
    if(argc < 2)
    {
        printf("Usage: ./RILO ../config/rilo_params.yaml \n");
        // return -1;
        config_file = "../config/rilo_params.yaml";
    }
    else{
        config_file = argv[1];
    }
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    
    // Initialize global params
    fsSettings["image_width"]  >> IMAGE_WIDTH;
    fsSettings["image_height"] >> IMAGE_HEIGHT;
    fsSettings["image_crop"]   >> IMAGE_CROP;
    fsSettings["use_brief"]    >> USE_BRIEF;
    fsSettings["use_orb"]      >> USE_ORB;
    fsSettings["num_bri_features"] >> NUM_BRI_FEATURES;
    fsSettings["num_orb_features"] >> NUM_ORB_FEATURES;
    fsSettings["min_loop_feature_num"] >> MIN_LOOP_FEATURE_NUM;
    fsSettings["min_loop_search_gap"]  >> MIN_LOOP_SEARCH_GAP;
    fsSettings["min_loop_search_time"] >> MIN_LOOP_SEARCH_TIME;
    fsSettings["min_loop_bow_th"]      >> MIN_LOOP_BOW_TH;
    fsSettings["skip_time"]    >> SKIP_TIME;
    fsSettings["num_threads"]  >> NUM_THREADS;
    fsSettings["debug_image"]  >> DEBUG_IMAGE;
    fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;

    std::string brief_pattern_file;
    fsSettings["brief_pattern_file"] >> brief_pattern_file;  
    briefExtractor = BriefExtractor(brief_pattern_file);

    // create a mask for blocking feature extraction
    MASK = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < IMAGE_HEIGHT; ++i)
        for (int j = 0; j < IMAGE_WIDTH; ++j)
            if (j < IMAGE_CROP || j > IMAGE_WIDTH - IMAGE_CROP)
                MASK.at<uchar>(i,j) = 0;
    
    KeyFrame* old_kf = nullptr;
    int global_frame_index = 0;
    for(int i = 0; i < 15; i++)
    {
        std::string fileName = "/home/zhihui/projects/RILO/data/pcd/";
        boost::format fmt("%s%06d.pcd");
        fmt %fileName % i;
        RILO::DataReader<PointType> data;
        pcl::PointCloud<PointType>::Ptr cloud;
        cloud.reset(new pcl::PointCloud<PointType>);
        data.readCloudFile(fmt.str(), cloud);
        
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        cv::Mat range_img;
        cv::Mat intensity_img;
        ProjLidar2Img<PointType> proj_lidar;
        proj_lidar.rangeProjection(cloud);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used =  std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
        std::cout << "lidar proj cost [" << time_used.count() * 1000 << "] ms" << std::endl;
        
        range_img = proj_lidar.getRangeImg();
        intensity_img = proj_lidar.getIntensityImg();
        cv::imshow("range img", range_img);
        cv::imshow("intensity img", intensity_img);
        cv::waitKey(10);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double cloud_time = 0;
        
        KeyFrame* keyframe = new KeyFrame(cloud_time,
                                        global_frame_index,
                                        intensity_img,
                                        cloud); 

        if(global_frame_index == 0)
        {
            old_kf = keyframe;
            global_frame_index++;
            continue;
        }
        
        keyframe->findConnection(old_kf);
        old_kf = keyframe;
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        time_used =  std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
        std::cout << "find feature point pairs cost [" << time_used.count() * 1000 << "] ms" << std::endl;
        global_frame_index++;
        delete keyframe;
    }

    delete old_kf;
    return 0;
}