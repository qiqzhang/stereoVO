# stereoVO
A Simple Stereo Visual Odometery


StereoVO is a real-time stereo visual odometry based part of ORB-SLAM2 and slambook/project.It is developed for the final project of SLAM course.It runs on laptops with CPU and only provides localization services for self-driving cars.

### Authors: Xiaoqiang JIANG

### 1 Installation

#### 1.1 Required Dependencies

- OpenCV3.x:  sudo apt-get install libopencv-dev(or from source)
- g2o  :sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake
- eigen3  :sudo apt-get install libeigen3-dev
- Pangolin(for visualization): <<https://github.com/stevenlovegrove/Pangolin>/>

Please follow  :   <https://github.com/raulmur/ORB_SLAM2/>

#### 1.2 Build

> cd stereoVO
>
> mkdir build
>
> cd build 
>
> cmake ..
>
> make -j



### 2 Usage

Run on kitti dataset from:  <http://www.cvlibs.net/datasets/kitti/eval_object.php/>

You should modify the dataset _dir in KITTI xx-xx.yaml in the source directory

And run ./vo_pangolin under bin.

### 3 Experiments

It is evaluated by evo tools(<https://github.com/MichaelGrupp/evo/>)

![1](C:\Users\Administrator\Desktop\SLAM作业\大作业\stereoVO_data\1.png )





![2](C:\Users\Administrator\Desktop\SLAM作业\大作业\stereoVO_data\2.png)



![r_2](C:\Users\Administrator\Desktop\SLAM作业\大作业\stereo_VO\station_tests\r_2.png)

![map_2](C:\Users\Administrator\Desktop\SLAM作业\大作业\stereo_VO\station_tests\map_2.png)
