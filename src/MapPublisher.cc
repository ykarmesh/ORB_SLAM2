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

#include "MapPublisher.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "Tracking.h"
namespace ORB_SLAM2
{


MapPublisher::MapPublisher(Map* pMap):mpMap(pMap), mbCameraUpdated(false)
{
	cout<<"Configure KeyFrames"<<endl;
	mKeyFrames.header.frame_id = MAP_FRAME_ID;
	mKeyFrames.ns = KEYFRAMES_NAMESPACE;
	mKeyFrames.id=1;
	mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
	mKeyFrames.scale.x=0.005;
	mKeyFrames.pose.orientation.w=1.0;
	mKeyFrames.action=visualization_msgs::Marker::ADD;
	mKeyFrames.color.b=1.0f;
	mKeyFrames.color.a = 1.0;

	cout<<"Configure Covisibility Graph"<<endl;
	mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
	mCovisibilityGraph.ns = GRAPH_NAMESPACE;
	mCovisibilityGraph.id=2;
	mCovisibilityGraph.type = visualization_msgs::Marker::LINE_LIST;
	mCovisibilityGraph.scale.x=0.002;
	mCovisibilityGraph.pose.orientation.w=1.0;
	mCovisibilityGraph.action=visualization_msgs::Marker::ADD;
	mCovisibilityGraph.color.b=0.7f;
	mCovisibilityGraph.color.g=0.7f;
	mCovisibilityGraph.color.a = 0.3;

	cout<<"Configure KeyFrames Spanning Tree"<<endl;
	mMST.header.frame_id = MAP_FRAME_ID;
	mMST.ns = GRAPH_NAMESPACE;
	mMST.id=3;
	mMST.type = visualization_msgs::Marker::LINE_LIST;
	mMST.scale.x=0.03;
	mMST.pose.orientation.w=1.0;
	mMST.action=visualization_msgs::Marker::ADD;
	mMST.color.b=0.0f;
	mMST.color.g=1.0f;
	mMST.color.a = 1.0;

	cout<<"Configure MapPoint"<<endl;

	mMapPointCloud.header.frame_id= MAP_FRAME_ID;
	mMapPointCloud.header.stamp=ros::Time::now();
	mMapPointCloud.header.seq=0;
	mMapPointCloud.fields.resize(6);
	mMapPointCloud.fields[0].name = "x";
	mMapPointCloud.fields[0].offset = 0*sizeof(uint32_t);
	mMapPointCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	mMapPointCloud.fields[0].count = 1;
	mMapPointCloud.fields[1].name = "y";
	mMapPointCloud.fields[1].offset = 1*sizeof(uint32_t);
	mMapPointCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	mMapPointCloud.fields[1].count = 1;
	mMapPointCloud.fields[2].name = "z";
	mMapPointCloud.fields[2].offset = 2*sizeof(uint32_t);
	mMapPointCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	mMapPointCloud.fields[2].count = 1;
	mMapPointCloud.fields[3].name = "rgb";
	mMapPointCloud.fields[3].offset = 3*sizeof(uint32_t);
	mMapPointCloud.fields[3].datatype = sensor_msgs::PointField::INT32;
	mMapPointCloud.fields[3].count = 1;
	mMapPointCloud.fields[4].name = "KF";
	mMapPointCloud.fields[4].offset = 4*sizeof(uint32_t);
	mMapPointCloud.fields[4].datatype = sensor_msgs::PointField::INT32;
	mMapPointCloud.fields[4].count = 1;
	mMapPointCloud.fields[5].name = "lvl";
	mMapPointCloud.fields[5].offset = 5*sizeof(uint32_t);
	mMapPointCloud.fields[5].datatype = sensor_msgs::PointField::INT32;
	mMapPointCloud.fields[5].count = 1;
	mMapPointCloud.point_step = 6*sizeof(uint32_t);
	mMapPointCloud.is_dense = false;

	cout<<"Configure Publishers"<<endl;
	mapPointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("ORB_SLAM2/Pointcloud",1);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM2/Pose",1);
	kfPath_pub = nh.advertise<visualization_msgs::Marker>("ORB_SLAM2/Path",1);
	kfMST_pub = nh.advertise<visualization_msgs::Marker>("ORB_SLAM2/MST",1);
	kfCovisibility_pub = nh.advertise<visualization_msgs::Marker>("ORB_SLAM2/Covisibility",1);
}

void MapPublisher::Refresh(int state)
{
	if(state!=Tracking::OK)
		return;
	if(isCamUpdated())
	{
		PublishCurrentCamera(mCameraPose.clone());
		ResetCamFlag();
	}

	vector<KeyFrame*> vKeyFrames;
	vector<MapPoint*> vMapPoints;
	vector<MapPoint*> vRefMapPoints;

	{
		unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
		vKeyFrames = mpMap->GetAllKeyFrames();
		vMapPoints = mpMap->GetAllMapPoints();
		vRefMapPoints = mpMap->GetReferenceMapPoints();
	}

	PublishMapPoints(vMapPoints, vRefMapPoints, mCameraPose.clone());
	PublishKeyFrames(vKeyFrames);
}

void MapPublisher::PublishMapPoints(const std::vector<MapPoint*> &vpMPs, const std::vector<MapPoint*> &vpRefMPs, const cv::Mat &Tcw)
{
	cv::Mat TWC = Tcw.inv();
	cv::Mat tWC= TWC.rowRange(0,3).col(3);

	set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
	mMapPointCloud.header.seq++;
	mMapPointCloud.header.stamp = ros::Time::now();
	mMapPointCloud.data.clear();
	mMapPointCloud.height=1;
	mMapPointCloud.width=vpMPs.size();
	mMapPointCloud.row_step = mMapPointCloud.point_step * mMapPointCloud.width;
	mMapPointCloud.data.resize(mMapPointCloud.row_step * mMapPointCloud.height);
	unsigned char* dat = &(mMapPointCloud.data[0]);

	for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
	{
		if(vpMPs[i]->isBad())
			continue;
		cv::Mat pos = vpMPs[i]->GetWorldPos();
		float x = pos.at<float>(0);// - tWC.at<float>(0);
		float y = pos.at<float>(1);// - tWC.at<float>(1);
		float z = pos.at<float>(2);// - tWC.at<float>(2);
		/*new_pos = pos;
		new_pos.at<float>(1) = -pos.at<float>(2);
		new_pos.at<float>(2) = -pos.at<float>(1);*/
		uint32_t colorlvl = 0xff<<((8-vpMPs[i]->mnTrackScaleLevel)*3);
		uint32_t lvl = vpMPs[i]->mnTrackScaleLevel;
		uint32_t KF = vpMPs[i]->mnFirstKFid;
		memcpy(dat, &x,sizeof(float));
		memcpy(dat+sizeof(float), &y,sizeof(float));
		memcpy(dat+2*sizeof(float), &z,sizeof(float));
		/*memcpy(dat, &x,sizeof(float));
		memcpy(dat+sizeof(float), &y,sizeof(float));
		memcpy(dat+2*sizeof(float), &z,sizeof(float));
		*/memcpy(dat+3*sizeof(uint32_t),&colorlvl,sizeof(uint32_t));
		memcpy(dat+4*sizeof(uint32_t),&lvl,sizeof(uint32_t));
		memcpy(dat+5*sizeof(uint32_t),&KF,sizeof(uint32_t));
		dat+=mMapPointCloud.point_step;
	}

	mapPointCloud_pub.publish(mMapPointCloud);
}

void MapPublisher::PublishKeyFrames(const std::vector<KeyFrame*> &vpKFs)
{
	mKeyFrames.points.clear();
	mCovisibilityGraph.points.clear();
	mMST.points.clear();

	for(size_t i=0, iend=vpKFs.size() ;i<iend; i++)
	{
		cv::Mat kfOriginMat = vpKFs[i]->GetCameraCenter();

		geometry_msgs::Point kfOrigin;
		kfOrigin.x=kfOriginMat.at<float>(0);
		kfOrigin.y=kfOriginMat.at<float>(1);
		kfOrigin.z=kfOriginMat.at<float>(2);

		mKeyFrames.points.push_back(kfOrigin);

		// Covisibility Graph
		vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
		if(!vCovKFs.empty())
		{
			for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
			{
				if((*vit)->mnId<vpKFs[i]->mnId)
					continue;
				cv::Mat covKFOriginMat = (*vit)->GetCameraCenter();
				geometry_msgs::Point covKFOrigin;
				covKFOrigin.x=covKFOriginMat.at<float>(0);
				covKFOrigin.y=covKFOriginMat.at<float>(1);
				covKFOrigin.z=covKFOriginMat.at<float>(2);
				mCovisibilityGraph.points.push_back(kfOrigin);
				mCovisibilityGraph.points.push_back(covKFOrigin);
			}
		}

		// MST
		KeyFrame* pParent = vpKFs[i]->GetParent();
		if(pParent)
		{
			cv::Mat parentKFOriginMat = pParent->GetCameraCenter();
			geometry_msgs::Point parentKFOrigin;
			parentKFOrigin.x=parentKFOriginMat.at<float>(0);
			parentKFOrigin.y=parentKFOriginMat.at<float>(1);
			parentKFOrigin.z=parentKFOriginMat.at<float>(2);
			mMST.points.push_back(kfOrigin);
			mMST.points.push_back(parentKFOrigin);
		}
		set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
		for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
		{
			if((*sit)->mnId<vpKFs[i]->mnId)
				continue;
			cv::Mat kfLoopOriginMat = (*sit)->GetCameraCenter();
			geometry_msgs::Point kfLoopOrigin;
			kfLoopOrigin.x=kfLoopOriginMat.at<float>(0);
			kfLoopOrigin.y=kfLoopOriginMat.at<float>(1);
			kfLoopOrigin.z=kfLoopOriginMat.at<float>(2);
			mMST.points.push_back(kfOrigin);
			mMST.points.push_back(kfLoopOrigin);
		}
	}

	mKeyFrames.header.stamp = ros::Time::now();
	mCovisibilityGraph.header.stamp = ros::Time::now();
	mMST.header.stamp = ros::Time::now();

	kfPath_pub.publish(mKeyFrames);
	kfCovisibility_pub.publish(mCovisibilityGraph);
	kfMST_pub.publish(mMST);
}

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw)
{
	cv::Mat TWC = Tcw.inv();
	cv::Mat RWC= TWC.rowRange(0,3).colRange(0,3);
	cv::Mat tWC= TWC.rowRange(0,3).col(3);

	tf::Matrix3x3 M(RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
		RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
		RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2));
	tf::Vector3 V(tWC.at<float>(0), tWC.at<float>(1), tWC.at<float>(2));

	tf::StampedTransform transformco;
	try
	{
        	listener.lookupTransform("/camera_rgb_optical_frame", "/odom", ros::Time(0), transformco);
	}
  	catch (tf::TransformException &ex)
	{
  		ROS_ERROR("%s",ex.what());
		return;
  	}

	static tf::TransformBroadcaster br;
	tf::Transform transformwc = tf::Transform(M, V);
	br.sendTransform(tf::StampedTransform(transformwc * transformco, ros::Time::now(), MAP_FRAME_ID, ODOM_FRAME_ID));
	geometry_msgs::PoseStamped _pose;
	_pose.pose.position.x = transformwc.getOrigin().x();
	_pose.pose.position.y = transformwc.getOrigin().y();
	_pose.pose.position.z = transformwc.getOrigin().z();
	_pose.pose.orientation.x = transformwc.getRotation().x();
	_pose.pose.orientation.y = transformwc.getRotation().y();
	_pose.pose.orientation.z = transformwc.getRotation().z();
	_pose.pose.orientation.w = transformwc.getRotation().w();

	_pose.header.stamp = ros::Time::now();
	_pose.header.frame_id = MAP_FRAME_ID;
	pose_pub.publish(_pose);
}

void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)
{
	boost::mutex::scoped_lock lock(mMutexCamera);
	mCameraPose = Tcw.clone();
	mbCameraUpdated = true;
}

bool MapPublisher::isCamUpdated()
{
	boost::mutex::scoped_lock lock(mMutexCamera);
	return mbCameraUpdated;
}

void MapPublisher::ResetCamFlag()
{
	boost::mutex::scoped_lock lock(mMutexCamera);
	mbCameraUpdated = false;
}

} //namespace ORB_SLAM2
