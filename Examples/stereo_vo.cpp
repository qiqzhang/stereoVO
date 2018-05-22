// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "config.h"
#include "visual_odometry.h"
#include <boost/format.hpp>

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    myslam::Config::setParameterFile ( argv[1] );
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/times.txt" );
    if ( !fin )
    {
        cout<<"please open this file in the 05 file!"<<endl;
        return 1;
    }

    vector<string> left_files, right_files;
    vector<double> left_times;
    while ( !fin.eof() )
    {
        string left_time;
        fin>>left_time;
        left_times.push_back ( atof ( left_time.c_str() ) );
        if ( fin.good() == false )
            break;
    }
    for(int i=0; i < left_times.size();i++){
        boost::format fmt("%s/%s/%06d.%s");
        left_files.push_back((fmt%dataset_dir%"image_0"%i%"png").str());
        right_files.push_back((fmt%dataset_dir%"image_1"%i%"png").str());

    }

    myslam::Camera::Ptr camera ( new myslam::Camera );

    // visualization
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

    cout<<"read total "<<left_files.size() <<" entries"<<endl;
    for ( int i=0; i<left_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat leftImg = cv::imread ( left_files[i] ,CV_8UC1);
        Mat rightImg = cv::imread ( right_files[i],CV_8UC1);
        if ( leftImg.data==nullptr || rightImg.data==nullptr )
            break;
        boost::timer timer;
        shared_ptr<myslam::ORBextractor> leftORBextractor(new myslam::ORBextractor(vo->num_of_features_,vo->scale_factor_,
                                                                                   vo->level_pyramid_,vo->iniThFAST,vo->minThFAST));
        shared_ptr<myslam::ORBextractor> rightORBextractor(new myslam::ORBextractor(vo->num_of_features_,vo->scale_factor_,
                                                                                   vo->level_pyramid_,vo->iniThFAST,vo->minThFAST));
        myslam::Frame::Ptr pFrame(new myslam::Frame(leftImg,rightImg,leftORBextractor.get(),
                                                    rightORBextractor.get(),vo->ThDepth_,left_times[i],camera));


        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
        SE3 Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        Mat img_show = leftImg.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::viz::WCameraPosition cpw(0.1); // Coordinate axes
        cv::viz::WCameraPosition cpw_frustum(cv::Matx33d(camera->K), img_show, 0.1, cv::viz::Color::white()); // Camera frustum
        string widgetPoseName = "CPW" + i;
        string widgetFrustumName = "CPW_FRUSTUM" + i;
        vis.showWidget(widgetPoseName, cpw, M);
        vis.showWidget(widgetFrustumName, cpw_frustum, M);

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false);
        cout<<endl;
    }

    return 0;
}
