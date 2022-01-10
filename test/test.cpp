#include "iostream"
#include "data_reader.hpp"
#include "lidar_proj.hpp"
#include "keyframe.hpp"
#include "common.h"
#include "viewer.h"

#include <teaser/registration.h>

#include <boost/format.hpp>

Viewer view;

std::string DATA_PATH;
int START_FRAME;
int END_FRAME;
int IMAGE_WIDTH;
int IMAGE_HEIGHT;
int IMAGE_CROP;
int USE_BRIEF;
int USE_ORB;
int NUM_BRI_FEATURES;
int NUM_ORB_FEATURES;
int MIN_LOOP_FEATURE_NUM;
double SKIP_TIME = 0;
int NUM_THREADS;
int DEBUG_IMAGE;
double MATCH_IMAGE_SCALE;
cv::Mat MASK;
BriefExtractor briefExtractor;

void save_pcd(std::string save_path, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud = *cloud_in;
    cloud.width    = cloud_in->size(); 
    cloud.height   = 1; 
    cloud.is_dense = false;
    pcl::io::savePCDFileASCII (save_path, cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
}

int main(int argc, char* argv[])
{
    std::string config_file;
    if(argc < 2)
    {
        printf("Usage: ./RILO ../config/rilo_params.yaml \n");
        // return -1;
        config_file = "/home/zhihui/projects/RILO/config/rilo_params.yaml";
    }
    else{
        config_file = argv[1];
    }
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    
    // Initialize global params
    fsSettings["start_frame"]  >> START_FRAME;
    fsSettings["end_frame"] >> END_FRAME;
    fsSettings["image_width"]  >> IMAGE_WIDTH;
    fsSettings["image_height"] >> IMAGE_HEIGHT;
    fsSettings["image_crop"]   >> IMAGE_CROP;
    fsSettings["use_brief"]    >> USE_BRIEF;
    fsSettings["use_orb"]      >> USE_ORB;
    fsSettings["num_bri_features"] >> NUM_BRI_FEATURES;
    fsSettings["num_orb_features"] >> NUM_ORB_FEATURES;
    fsSettings["min_loop_feature_num"] >> MIN_LOOP_FEATURE_NUM;
    fsSettings["skip_time"]    >> SKIP_TIME;
    fsSettings["num_threads"]  >> NUM_THREADS;
    fsSettings["debug_image"]  >> DEBUG_IMAGE;
    fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;
    fsSettings["data_path"]  >> DATA_PATH;

    std::string brief_pattern_file;
    fsSettings["brief_pattern_file"] >> brief_pattern_file;  
    briefExtractor = BriefExtractor(brief_pattern_file);

    // create a mask for blocking feature extraction
    MASK = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < IMAGE_HEIGHT; ++i)
        for (int j = 0; j < IMAGE_WIDTH; ++j)
            if (j < IMAGE_CROP || j > IMAGE_WIDTH - IMAGE_CROP)
                MASK.at<uchar>(i,j) = 0;

    int global_frame_index = 0;
    KeyFrame* old_kf = nullptr;
    
    for(int i = START_FRAME; i < END_FRAME; i++)
    {
        printf("\n frame [%d]\n", i);
        std::string fileName = DATA_PATH;
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
        proj_lidar.rangeProjection(cloud, IMAGE_HEIGHT, IMAGE_WIDTH);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used =  std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
        std::cout << "lidar proj cost [" << time_used.count() * 1000 << "] ms" << std::endl;
        
        range_img = proj_lidar.getRangeImg();
        intensity_img = proj_lidar.getIntensityImg();
        cv::imshow("range img", range_img);
        cv::imshow("intensity img", intensity_img);
        cv::waitKey(1);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double cloud_time = 0;
        
        KeyFrame* keyframe = new KeyFrame(cloud_time,
                                        global_frame_index,
                                        intensity_img,
                                        range_img,
                                        proj_lidar.getPointCloudAfterProcess()); 

        if(global_frame_index == 0)
        {
            old_kf = keyframe;
            global_frame_index++;
            continue;
        }
        
        keyframe->findConnection(old_kf);
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        time_used =  std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
        std::cout << "find feature point pairs cost [" << time_used.count() * 1000 << "] ms" << std::endl;
        global_frame_index++;

        std::cout << "cur cloud matched: " << keyframe->cur_cloud_matched->size() << std::endl;
        std::cout << "pre cloud matched: " << keyframe->pre_cloud_matched->size() << std::endl;

        if(0)
        {
            save_pcd("/home/zhihui/projects/RILO/results/cur_matched_cloud.pcd", keyframe->cur_cloud_matched);
            save_pcd("/home/zhihui/projects/RILO/results/pre_matched_cloud.pcd", keyframe->pre_cloud_matched);
        }

        // registration
        // frame to frame
        std::chrono::steady_clock::time_point t4, t5;
        t4 = std::chrono::steady_clock::now();
        pcl::PointCloud<PointCloudXYZIRCR> src_tmp;
        pcl::PointCloud<PointCloudXYZIRCR> tgt_tmp;
        src_tmp.points.resize(keyframe->cur_cloud_matched->size());
        tgt_tmp.points.resize(keyframe->cur_cloud_matched->size());
        for(int i = 0; i < keyframe->cur_cloud_matched->size(); i++)
        {
            // PointCloudXYZIRCR pt_src;
            src_tmp.points[i].x = keyframe->cur_cloud_matched->points[i].x;
            src_tmp.points[i].y = keyframe->cur_cloud_matched->points[i].y;
            src_tmp.points[i].z = keyframe->cur_cloud_matched->points[i].z;

            tgt_tmp.points[i].x = keyframe->pre_cloud_matched->points[i].x;
            tgt_tmp.points[i].y = keyframe->pre_cloud_matched->points[i].y;
            tgt_tmp.points[i].z = keyframe->pre_cloud_matched->points[i].z;
        }

        int N = src_tmp.size();
        // Convert the point cloud to Eigen
        Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
        Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);
        for (int i = 0; i < N; ++i)
        {
            src.col(i) << src_tmp.points[i].x, src_tmp.points[i].y, src_tmp.points[i].z;
            tgt.col(i) << tgt_tmp.points[i].x, tgt_tmp.points[i].y, tgt_tmp.points[i].z;
        }

        // Run TEASER++ registration
        // Prepare solver parameters
        teaser::RobustRegistrationSolver::Params params;
        params.noise_bound = 0.2;
        params.cbar2 = 1.0;
        params.estimate_scaling = false;
        params.rotation_max_iterations = 100;
        params.rotation_gnc_factor = 1.4;
        params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
            // teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::FGR;
        params.use_max_clique = true;
        params.kcore_heuristic_threshold = 0.5;
        params.rotation_cost_threshold = 0.005;

        // Solve with TEASER++
        teaser::RobustRegistrationSolver solver(params);
        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        solver.solve(src, tgt);
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        auto solution = solver.getSolution();
        std::vector<int> rot_inliers;
        std::vector<int> trans_inliers;
        rot_inliers = solver.getRotationInliers();
        trans_inliers = solver.getTranslationInliers();
        std::cout << "Inlier correspondences found. [" << rot_inliers.size() << "]   ["  << trans_inliers.size() << "] " <<std::endl;
        
        Eigen::Matrix4d trans_matrix_ = Eigen::Matrix4d::Identity();
        trans_matrix_.block(0, 0, 3, 3) = solution.rotation;
        trans_matrix_.block(0, 3, 3, 1) = solution.translation;
        std::cout << "trans:\n" << trans_matrix_ << std::endl;

        t5 = std::chrono::steady_clock::now();
        std::cout << "[TEASER++ Compute Time] taken (ms): "
                << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() / 1000.0 << std::endl;

        view.setPCDSource(*(keyframe->cloud));
        view.setPCDTarget(*(old_kf->cloud));
        view.setPCDMatchedPointPairs(src_tmp, tgt_tmp);

        pcl::PointCloud<PointCloudXYZIRCR>::Ptr src_tranformed(new pcl::PointCloud<PointCloudXYZIRCR>);
        pcl::transformPointCloud(*(keyframe->cloud), *src_tranformed, trans_matrix_);
        view.setPCDTransformed(*src_tranformed);
        view.updateMap();

        *old_kf = *keyframe;
        keyframe->freeMemory();
        old_kf->freeMemory();    
    }

    std::cout << "finished" << std::endl;
    return 0;
}