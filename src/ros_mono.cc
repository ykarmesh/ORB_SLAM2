/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include"System.h"

using namespace std;

class ROSCallbacks
{
public:
    ROSCallbacks(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void ImageCb(const sensor_msgs::ImageConstPtr& msg);
    void OdomCb(const nav_msgs::OdometryPtr& msg);
    void StringCb(const std_msgs::StringPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ROSCallbacks cb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber img_sub = nodeHandler.subscribe("/camera/rgb/image_raw", 1, &ROSCallbacks::ImageCb,&cb);
    ros::Subscriber odom_sub = nodeHandler.subscribe("/odom", 1, &ROSCallbacks::OdomCb,&cb);
    ros::Subscriber reset_sub = nodeHandler.subscribe("/ORB_SLAM2/Reset", 1, &ROSCallbacks::StringCb,&cb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ROSCallbacks::ImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


void ROSCallbacks::OdomCb(const nav_msgs::OdometryPtr& msg)
{
    mpSLAM->SetOdometry(msg);
}

void ROSCallbacks::StringCb(const std_msgs::StringPtr& msg)
{
    mpSLAM->Reset();
}