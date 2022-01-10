#ifndef _VIEWER_H_
#define _VIEWER_H_

#include <thread>
#include <iostream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "common.h"

using namespace pcl;

class Viewer
{
    using PointType = PointCloudXYZIRCR;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();
    ~Viewer();

    void setPCDSource(pcl::PointCloud<PointType> &pcd_src){pcd_src_ = pcd_src;}
    void setPCDTarget(pcl::PointCloud<PointType> &pcd_tgt){pcd_tgt_ = pcd_tgt;}

    void setPCDMatchedPointPairs(pcl::PointCloud<PointType> &pcd_src, pcl::PointCloud<PointType> &pcd_tgt)
    {
        pcd_src_matched_ = pcd_src;
        pcd_tgt_matched_ = pcd_tgt;
    }

    void setPCDTransformed(pcl::PointCloud<PointType> &pcd_transformed){ pcd_transformed_ = pcd_transformed;}

    void setCurrentPose(Eigen::Vector3d &p, Eigen::Quaterniond &q)
    {
        cur_p_ = p;
        cur_q_ = q;
    }

    void close();

    void updateMap();

private:
    void threadLoop();

    void drawPoints(pcl::PointCloud<PointType> &pcd, const float color[3]);
    void drawLines(pcl::PointCloud<PointType> &pcd_src_mathced, pcl::PointCloud<PointType> &pcd_tgt_mathced, const float color[3]);

    void drawTrajectory();

    void followCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

    void getCurrentAxisPose(pangolin::OpenGlMatrix &M);

    std::thread viewer_thread_;

    bool map_updated_ = false;

    bool viewer_running_ = true;

    std::mutex viewer_data_mutex_;

    std::vector<Eigen::Vector3d> traj;


    pcl::PointCloud<PointType> pcd_src_;
    pcl::PointCloud<PointType> pcd_tgt_;
    pcl::PointCloud<PointType> pcd_src_matched_;
    pcl::PointCloud<PointType> pcd_tgt_matched_;
    pcl::PointCloud<PointType> pcd_transformed_;

    Eigen::Vector3d cur_p_, opt_p_;
    Eigen::Quaterniond cur_q_, opt_q_;

    bool draw_opt_traj = false;
};

#endif //_POINT_VIEWER_H_