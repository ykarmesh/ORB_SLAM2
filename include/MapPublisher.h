/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>
//#include <pangolin/pangolin.h>

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM2
{

class MapPublisher
{
public:
	MapPublisher(Map* pMap);

	Map* mpMap;

	void Refresh(int state);
	void SetCurrentCameraPose(const cv::Mat &Tcw);

private:

    const char* MAP_FRAME_ID = "world";
    const char* ODOM_FRAME_ID = "odom";
    const char* POINTS_NAMESPACE = "MapPoints";
    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    const char* GRAPH_NAMESPACE = "Graph";
    const char* CAMERA_NAMESPACE = "Camera";

	void PublishMapPoints(const std::vector<MapPoint*> &vpMPs, const std::vector<MapPoint*> &vpRefMPs, const cv::Mat &Tcw);
	void PublishKeyFrames(const std::vector<KeyFrame*> &vpKFs);
	void PublishCurrentCamera(const cv::Mat &Tcw);

	bool isCamUpdated();
	void ResetCamFlag();

	ros::NodeHandle nh;
	ros::Publisher publisher;
	ros::Publisher mapPointCloud_pub, pose_pub, kfPath_pub, kfMST_pub, kfCovisibility_pub;

	visualization_msgs::Marker mKeyFrames, mCovisibilityGraph, mMST;
	sensor_msgs::PointCloud2 mMapPointCloud;

    	float fCameraSize;
	float fPointSize;

	cv::Mat mCameraPose;
	bool mbCameraUpdated;
	int mState;
	boost::mutex mMutexCamera;

	tf::TransformListener listener;
};

} //namespace ORB_SLAM2

#endif // MAPPUBLISHER_H
