#include "MapDrawer.h"
#include <opencv/cv.hpp>

namespace myslam
{


MapDrawer::MapDrawer()
{

    mImageWidth = Config::get<float>("Camera.width");
    mImageHeight = Config::get<float>("Camera.height");
    mPointSize = Config::get<float>("Viewer.PointSize");
    mCameraSize = Config::get<float>("Viewer.CameraSize");
    mCameraLineWidth = Config::get<float>("Viewer.CameraLineWidth");
    float fps = Config::get<double>("Camera.fps");
    mT = 1e3/fps;
    mViewpointX = Config::get<float>("Viewer.ViewpointX");
    mViewpointY = Config::get<float>("Viewer.ViewpointY");
    mViewpointZ = Config::get<float>("Viewer.ViewpointZ");
    mViewpointF = Config::get<float>("Viewer.ViewpointF");

}

//关于gl相关的函数，可直接google, 并加上msdn关键词
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    //相机模型大小：宽度占总宽度比例为0.08
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    //百度搜索：glPushMatrix 百度百科
    glPushMatrix();

    //将4*4的矩阵Twc.m右乘一个当前矩阵
    //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
    //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    //设置绘制图形时线的宽度
    glLineWidth(mCameraLineWidth);
    //设置当前颜色为绿色(相机图标显示为绿色)
    glColor3f(0.0f,1.0f,0.0f);
    //用线将下面的顶点两两相连
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}
void MapDrawer::DrawTrajectory() {

    unique_lock<mutex> lock(mMutextwc);
    for(int i = 1;i < twcs.size();i++) {
        if (i >= 1) {
            glLineWidth(1.5);
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            Eigen::Vector3d twc = twcs[i - 1];
            Eigen::Vector3d twc2 = twcs[i];
            glVertex3f(twc(0), twc(1), twc(2));
            glVertex3f(twc2(0), twc2(1), twc2(2));
            glEnd();
        }
    }
}
    //绘制局部地图点
void MapDrawer::DrawCurrentMappoints() {
    unique_lock<mutex> lock(mMutexPoint);
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for(auto iter = map_points.begin();iter != map_points.end();iter++){
        Eigen::Vector3d pos= iter->second->pos_;
        glVertex3d(pos(0),pos(1),pos(2));
    }
    glEnd();
}
   //绘制坐标系
void MapDrawer::DrawCoordinateSystem() {


        float planeLength = 2.5;

        float extraLength = 1;


        int n = 10;   //num of grid


        float gridUnit = planeLength*2.0/n;

        float fontSize = 0.08;

        float offset = 2*fontSize+0.1;  //make the font not cover the axis

        glLineWidth(5);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        //x axis
        glVertex3f(planeLength+extraLength,0,0);
        glVertex3f(-(planeLength+extraLength),0,0);
        //y axis
        glVertex3f(0,planeLength+extraLength,0);
        glVertex3f(0,-(planeLength+extraLength),0);
        //z axis
        glVertex3f(0,0,(planeLength+extraLength));
        glVertex3f(0,0,-(planeLength+extraLength));
        glEnd();

        glLineWidth(2);
        glBegin(GL_LINES);
        // label x
        glColor4f(0.0f,0.0f,1.0f,0.6f);
        glVertex3f(planeLength+extraLength+fontSize*0.7,0+fontSize+offset,0);
        glVertex3f(planeLength+extraLength-fontSize*0.7,0-fontSize+offset,0);

        glVertex3f(planeLength+extraLength-fontSize*0.7,0+fontSize+offset,0);
        glVertex3f(planeLength+extraLength+fontSize*0.7,0-fontSize+offset,0);


        // label y
        glVertex3f(0+offset-fontSize*0.7,planeLength+extraLength+fontSize,0);
        glVertex3f(0+offset,planeLength+extraLength,0);

        glVertex3f(0+offset+fontSize*0.7,planeLength+extraLength+fontSize,0);
        glVertex3f(0+offset,planeLength+extraLength,0);

        glVertex3f(0+offset,planeLength+extraLength,0);
        glVertex3f(0+offset,planeLength+extraLength-fontSize*1.2,0);



        glColor4f(0.0f,0.0f,0.0f,0.6f);

        for (int i = 0; i < n+1; ++i) {
            glVertex3f(-planeLength,-planeLength+i*gridUnit,0);
            glVertex3f(planeLength,-planeLength+i*gridUnit,0);

            glVertex3f(-planeLength+i*gridUnit,-planeLength,0);
            glVertex3f(-planeLength+i*gridUnit,planeLength,0);
        }


        glEnd();


 }
//添加关键帧在世界坐标系中的位置数组
void MapDrawer::addKeyframePos(Eigen::Vector3d twc) {
    unique_lock<mutex> lock(mMutextwc);
    twcs.push_back(twc);
}


void MapDrawer::SetCurrentCameraPose(const SE3 &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw;
}

void MapDrawer::SetCurrentMappoints(unordered_map<unsigned long, MapPoint::Ptr> &mappoints) {
    unique_lock<mutex> lock(mMutexPoint);
    map_points = mappoints;
}

// 将相机位姿mCameraPose由Mat类型转化为OpenGlMatrix类型
void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    Eigen::Matrix3d Rwc;
    Eigen::Vector3d twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Rwc = mCameraPose.rotation_matrix().transpose();
        twc = -Rwc * mCameraPose.translation();
    }
    M.m[0] = Rwc(0,0);
    M.m[1] = Rwc(1,0);
    M.m[2] = Rwc(2,0);
    M.m[3]  = 0.0;

    M.m[4] = Rwc(0,1);
    M.m[5] = Rwc(1,1);
    M.m[6] = Rwc(2,1);
    M.m[7]  = 0.0;

    M.m[8] = Rwc(0,2);
    M.m[9] = Rwc(1,2);
    M.m[10] = Rwc(2,2);
    M.m[11]  = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15]  = 1.0;
}
void MapDrawer::Run() {
    pangolin::CreateWindowAndBind("stereoVO",1024,768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow ");


    // Define Camera Render Object (for view / scene browsing)
    // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
    // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
    //                观测目标位置：(0, 0, 0)
    //                观测的方位向量：(0.0,-1.0, 0.0)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
            pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
    // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    bool bFollow = true;
    while(!pangolin::ShouldQuit()){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow){
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow){
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,0,0,0,0.0,-1.0,0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow){
            bFollow = false;
        }
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        //DrawCoordinateSystem();
        DrawCurrentCamera(Twc);
        //绘制相机轨迹
        DrawTrajectory();
        DrawCurrentMappoints();
        pangolin::FinishFrame();
        cv::waitKey(mT);
    }



}
}
