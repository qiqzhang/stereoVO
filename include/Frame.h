
#ifndef FRAME_H
#define FRAME_H
#include "common_include.h"
#include "ORBextractor.h"
#include "camera.h"
#include <opencv2/opencv.hpp>

namespace myslam
{
// forward declare
    class MapPoint;
class Frame
{
public:
    Frame();

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight,
          ORBextractor* extractorLeft, ORBextractor* extractorRight,
           const float &thDepth ,const double time_stamp = 0,Camera::Ptr camera=nullptr,
          SE3 T_c_w=SE3());//缺省函数参数在头文件里面如果定义的话，就不用在类实现里面定义第二次
    ~Frame();


    // Get Camera Center
    Vector3d getCamCenter() const;

    void setPose( const SE3& T_c_w );

    // check if a point is in this frame
    bool isInFrame( const Vector3d& pt_world );
    // Extract ORB on the image. 0 for left image and 1 for right image.
    // 提取的关键点存放在mvKeys和mDescriptors中
    // ORB是直接调orbExtractor提取的
    void ExtractORB(int flag, const cv::Mat &im);

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

public:
    typedef std::shared_ptr<Frame> Ptr;
    double                         time_stamp_; // when it is recorded
    SE3                            T_c_w_;      // transform from world to camera
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model
    Mat                            color_;    //    left image
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    bool                           is_key_frame_;  // whether a key-frame

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;
    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;
    // Number of KeyPoints.
    int N; ///< KeyPoints数量

    // mvKeys:原始左图像提取出的特征点（未校正）
    // mvKeysRight:原始右图像提取出的特征点（未校正）
    // mvKeysUn:校正mvKeys后的特征点，对于双目摄像头，一般得到的图像都是校正好的，再校正一次有点多余
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    // 对于双目，mvuRight存储了左目像素点在右目中的对应点的横坐标
    // mvDepth对应的深度
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;
    // ORB descriptor, each row associated to a keypoint.
    // 左目摄像头和右目摄像头特征点对应的描述子
    cv::Mat mDescriptors, mDescriptorsRight;
    // Current and Next Frame id.
    static long unsigned int nNextId; ///< Next Frame id.
    long unsigned int id_; ///< Current Frame id.

    // Scale pyramid info.
    int mnScaleLevels;//图像提金字塔的层数
    float mfScaleFactor;//图像提金字塔的尺度因子
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;



};

}

#endif // FRAME_H
