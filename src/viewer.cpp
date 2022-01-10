#include "viewer.h"
#include <pangolin/pangolin.h>

Viewer::Viewer()
{
    viewer_thread_ = std::thread(std::bind(&Viewer::threadLoop, this));

    cur_p_ = Eigen::Vector3d(0, 0, 0);
    opt_p_ = Eigen::Vector3d(0, 0, 0);
    cur_q_ = Eigen::Quaterniond(1, 0, 0, 0);
    opt_q_ = Eigen::Quaterniond(1, 0, 0, 0);
}

Viewer::~Viewer()
{
    std::cout << "1"<<std::endl;
    if(viewer_thread_.joinable())
    {
        std::cout << "2"<<std::endl;
        viewer_running_ = false;
        viewer_thread_.join();
        std::cout << "3"<<std::endl;
    }
    std::cout << "4"<<std::endl;
}

void Viewer::close()
{
    viewer_running_ = false;
    if(viewer_thread_.joinable()){
        viewer_thread_.join();
    }
}

void Viewer::updateMap()
{
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    map_updated_ = true;
}

void Viewer::threadLoop()
{
    pangolin::CreateWindowAndBind("Lidar Relocalization Reg", 1280, 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(-0, 0, 30, 0, 0, 0, 0.0, 1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(200)); //new button and menu

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(210)); //创建

    pangolin::Var<bool> menuFollowCamera("ui.menuFollowCamera", true, true);        //设置一个按钮，默认值为false，最后一个false表示按钮形式
    pangolin::Var<bool> menuShowPCDSrc("ui.menuShowPCDSrc", true, true);     //设置一个按钮，默认值为false，最后一个false表示按钮形式
    pangolin::Var<bool> menuShowPCDTgt("ui.menuShowPCDTgt", true, true);
    pangolin::Var<bool> menuShowPointMatched("ui.menuShowPointMatched", true, true);     //设置一个按钮，默认值为false，最后一个false表示按钮形式
    pangolin::Var<bool> menuShowPCDTransformed("ui.menuShowPCDTransformed", false, true);

    // pangolin::Var<double> a_double("ui.A_Double", 3, 0, 5);//设置一个double的、可拖动变换值的玩意(不知道咋形容)！

    while (!pangolin::ShouldQuit() && viewer_running_)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (menuFollowCamera)
        {
            followCurrentFrame(vis_camera);
        }

        vis_display.Activate(vis_camera);

        glClearColor(1.0f,1.0f,1.0f,1.0f);// background color

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);

        if (draw_opt_traj)
            drawTrajectory();

        if (menuShowPCDSrc)
        {
            if (pcd_src_.points.size() > 0)
            {
                const float color[3] = {1, 0, 0};
                drawPoints(pcd_src_, color);
            }
        }

        if (menuShowPCDTgt)
        {
            if (pcd_tgt_.points.size() > 0)
            {
                const float color[3] = {0, 1, 0};
                drawPoints(pcd_tgt_, color);
            }
        }

        if (menuShowPointMatched)
        {
            if (pcd_src_matched_.points.size() > 0 && pcd_tgt_matched_.points.size())
            {
                const float color[3] = {0, 0, 1};
                drawLines(pcd_src_matched_, pcd_tgt_matched_, color);
            }
        }

        if (menuShowPCDTransformed)
        {
            if (pcd_transformed_.points.size() > 0)
            {
                const float color[3] = {0, 1, 1};
                drawPoints(pcd_transformed_, color);
            }
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
}

void Viewer::followCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
{
    pangolin::OpenGlMatrix Twc;
    getCurrentAxisPose(Twc);
    pangolin::OpenGlMatrix m(Twc);
    vis_camera.Follow(m, true);
}

void Viewer::getCurrentAxisPose(pangolin::OpenGlMatrix &M)
{
    if (cur_p_.data())
    {
        Eigen::Matrix3d R = cur_q_.toRotationMatrix();

        M.m[0] = R(0, 0); //.r00;
        M.m[1] = R(1, 0);
        M.m[2] = R(2, 0);
        M.m[3] = 0.0;

        M.m[4] = R(0, 1);
        M.m[5] = R(1, 1);
        M.m[6] = R(2, 1);
        M.m[7] = 0.0;

        M.m[8] = R(0, 2);
        M.m[9] = R(1, 2);
        M.m[10] = R(2, 2);
        M.m[11] = 0.0;

        M.m[12] = cur_p_.x();
        M.m[13] = cur_p_.y();
        M.m[14] = cur_p_.z();
        M.m[15] = 1.0;
    }
    else
    {
        M.SetIdentity();
    }
}

void Viewer::drawPoints(pcl::PointCloud<PointType> &pcd, const float color[3])
{
    glPointSize(1);
    glBegin(GL_POINTS);
    for (auto &p : pcd.points)
    {
        glColor3f(color[0], color[1], color[2]);
        glVertex3d(p.x, p.y, p.z);
    }
    glEnd();
}

void Viewer::drawLines(pcl::PointCloud<PointType> &pcd_src_mathced, 
                       pcl::PointCloud<PointType> &pcd_tgt_mathced, 
                       const float color[3])
{
    glPointSize(10);
    glBegin(GL_POINTS);
    for (int i = 0; i < pcd_src_mathced.points.size(); i++)
    {
        glColor3f(color[0], color[1], color[2]);
        glVertex3d(pcd_src_mathced[i].x, pcd_src_mathced[i].y, pcd_src_mathced[i].z);
        glVertex3d(pcd_tgt_mathced[i].x, pcd_tgt_mathced[i].y, pcd_tgt_mathced[i].z);
    }
    glEnd();

    glLineWidth(5);//设置线宽
    glBegin(GL_LINES);
    for(int i = 0; i < pcd_src_mathced.points.size(); i++){
        glColor3f(color[0], color[1], color[2]);  
        glVertex3f(pcd_src_mathced[i].x, pcd_src_mathced[i].y, pcd_src_mathced[i].z); 
        glVertex3f(pcd_tgt_mathced[i].x, pcd_tgt_mathced[i].y, pcd_tgt_mathced[i].z); 
    }
    glEnd();
}

void Viewer::drawTrajectory()
{
    const float green[3] = {0.0, 1.0, 0.0};

    traj.push_back(cur_p_);

    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(green[0], green[1], green[2]);
    for (size_t i = 0; i < traj.size() - 1; i++)
    {
        glVertex3d(traj[i].x(), traj[i].y(), traj[i].z());
        glVertex3d(traj[i + 1].x(), traj[i + 1].y(), traj[i + 1].z());
    }
    glEnd();
}
