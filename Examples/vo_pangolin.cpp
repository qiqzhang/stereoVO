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
    //保存poses
    ofstream ofout("15.txt");

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
        Mat img_show = leftImg.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        Eigen::Matrix3d Rwc ;
        Eigen::Vector3d twc ;
        Rwc = vo->T_c_w_estimated_.rotation_matrix().transpose();
        twc = -Rwc * vo->T_c_w_estimated_.translation();
        ofout << Rwc(0,0) << " "<< Rwc(0,1) << " "<< Rwc(0,2) << " "<<twc(0) << " "
              << Rwc(1,0) << " "<< Rwc(1,1) << " "<< Rwc(1,2) << " "<<twc(1) << " "
              << Rwc(2,0) << " "<< Rwc(2,1) << " "<< Rwc(2,2) << " "<<twc(2) << "\n";
        cout<<endl;
    }
    ofout.close();

    return 0;
}
