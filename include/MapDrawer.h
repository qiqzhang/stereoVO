
#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include<pangolin/pangolin.h>
#include "config.h"
#include "common_include.h"
#include <mutex>
#include "mappoint.h"
namespace myslam
{

class MapDrawer
{
public:
    MapDrawer();
    void SetCurrentCameraPose(const SE3 &Tcw);
    void Run();
    void addKeyframePos(Eigen::Vector3d);
    void SetCurrentMappoints(unordered_map<unsigned long, MapPoint::Ptr > &mappoints);


private:

    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    Sophus::SE3 mCameraPose;

    double mT;
    float mImageWidth,mImageHeight;
    float mViewpointX,mViewpointY,mViewpointZ,mViewpointF;
    std::mutex mMutexCamera;
    std::mutex mMutextwc;
    std::mutex mMutexPoint;
    vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> twcs;
    unordered_map<unsigned long, MapPoint::Ptr >  map_points;
private:
    void DrawCoordinateSystem();
    void DrawTrajectory();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void DrawCurrentMappoints();
};

}

#endif // MAPDRAWER_H
