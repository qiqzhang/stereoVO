#include <iostream>
#include <ORBextractor.h>
#include <config.h>
#include <Frame.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;
using namespace cv;


string leftname = "./left.png";
string rightname = "./right.png";
string disparityname = "./disparity.png";
int main(int argc, char** argv) {
    Mat leftimg = imread(leftname,IMREAD_GRAYSCALE);
    Mat rightimg = imread(rightname,IMREAD_GRAYSCALE);
    Mat disparity = imread(disparityname,0);
    myslam::Config::setParameterFile(argv[1]);
    const float fx = myslam::Config::get<float>("Camera.fx");
    const float fy = myslam::Config::get<float>("Camera.fy");
    const float cx = myslam::Config::get<float>("Camera.cx");
    const float cy = myslam::Config::get<float>("Camera.cy");
    const float bf = myslam::Config::get<float>("Camera.bf");
    const int nFeatures = myslam::Config::get<float>("ORBextractor.nFeatures");
    const float scaleFactor = myslam::Config::get<float>("ORBextractor.scaleFactor");
    const int nLevels = myslam::Config::get<float>("ORBextractor.nLevels");
    const int iniThFAST = myslam::Config::get<float>("ORBextractor.iniThFAST");
    const int minThFAST = myslam::Config::get<float>("ORBextractor.minThFAST");
    myslam::ORBextractor* mpORBextractorLeft,*mpORBextractorRight;
    mpORBextractorLeft = new myslam::ORBextractor(nFeatures,scaleFactor,nLevels,iniThFAST,minThFAST);
    mpORBextractorRight = new myslam::ORBextractor(nFeatures,scaleFactor,nLevels,iniThFAST,minThFAST);
    myslam::Frame stereoFrame(leftimg,rightimg,1,mpORBextractorLeft,mpORBextractorRight,bf,40,fx);

    for(int i=0;i < stereoFrame.mvuRight.size();i++){
        if(stereoFrame.mvuRight[i]!=-1){

            KeyPoint mLKey = stereoFrame.mvKeys[i];
            int ref_disparity = disparity.at<uchar>(mLKey.pt.y,mLKey.pt.x);
            int dis = mLKey.pt.x - stereoFrame.mvuRight[i];
            cout << dis - ref_disparity<< "    "<<i <<endl;

        }

    }









   /* ORBextractor* mpORBextractorLeft,*mpORBextractorRight;
    mpORBextractorLeft = new ORBextractor(500,1.2f,8,31,20);
    mpORBextractorRight = new ORBextractor(500,1.2f,8,31,20);
    vector<KeyPoint> mvKeys,mvKeysRight;
    Mat mDescriptors,mDescriptorsRight;
    (*mpORBextractorLeft)(leftimg,Mat(),mvKeys,mDescriptors);
    (*mpORBextractorRight)(rightimg,Mat(),mvKeysRight,mDescriptorsRight);
    vector<DMatch> matches;
    FlannBasedMatcher matcher(new flann::LshIndexParams(5,10,2));
    matcher.match(mDescriptors,mDescriptorsRight,matches);
    
    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < mDescriptors.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < mDescriptors.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }
    Mat img_match;
    drawMatches(leftimg,mvKeys,rightimg,mvKeysRight,good_matches,img_match);
    imshow("match point :",img_match);*/
    waitKey(0);


    return 0;
}