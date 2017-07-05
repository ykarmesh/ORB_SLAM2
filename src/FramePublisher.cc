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

#include "FramePublisher.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<boost/thread.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int16.h"
#include <MapPoint.h>
#include <iostream>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>


namespace ORB_SLAM2
{ 



FramePublisher::FramePublisher(Map *pMap)
{
	mState=Tracking::SYSTEM_NOT_READY;
	mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
	mbUpdated = true;
	SetMap(pMap);
	mImagePub = mNH.advertise<sensor_msgs::Image>("ORB_SLAM2/Frame",10,true);

	PublishFrame();
}  

void FramePublisher::SetMap(Map *pMap)
{
	mpMap = pMap;
}

void FramePublisher::Refresh()
{
	if(mbUpdated)
	{
		PublishFrame();
		mbUpdated = false;
		//FramePublisher::slam_info();
	}
}

cv::Mat FramePublisher::DrawFrame()
{   
	cv::Mat im;
	vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
	vector<int> vMatches; // Initialization: correspondeces with reference keypoints
	vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
	vector<MapPoint*> vMatchedMapPoints; // Tracked MapPoints in current frame

	int state; // Tracking state

	//Copy variable to be used within scoped mutex
	{
		boost::mutex::scoped_lock lock(mMutex);
		state=mState;
		if(mState==Tracking::SYSTEM_NOT_READY)
			mState=Tracking::NO_IMAGES_YET;

		mIm.copyTo(im);

		if(mState==Tracking::NOT_INITIALIZED)
		{			
			vCurrentKeys = mvCurrentKeys;
			vIniKeys = mvIniKeys;
			vMatches = mvIniMatches;
		}
		else if(mState==Tracking::OK)
		{
			vCurrentKeys = mvCurrentKeys;
			vMatchedMapPoints = mvpMatchedMapPoints;
		}
		else if(mState==Tracking::LOST)
		{
			vCurrentKeys = mvCurrentKeys;
		}
	} // destroy scoped mutex -> release

	if(im.channels()<3)
		cvtColor(im,im,CV_GRAY2BGR);

	//Draw
	if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
	{
		for(unsigned int i=0; i<vMatches.size(); i++)
		{
			if(vMatches[i]>=0)
			{
				cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
						cv::Scalar(0,255,0));
			}
		}		
	}
	else if(state==Tracking::OK) //TRACKING
	{
		mnTracked=0;
		const float r = 5;
		for(unsigned int i=0;i<vMatchedMapPoints.size();i++)
		{
			if(vMatchedMapPoints[i] || mvbOutliers[i])
			{
				cv::Point2f pt1,pt2;
				pt1.x=vCurrentKeys[i].pt.x-r;
				pt1.y=vCurrentKeys[i].pt.y-r;
				pt2.x=vCurrentKeys[i].pt.x+r;
				pt2.y=vCurrentKeys[i].pt.y+r;
				if(!mvbOutliers[i])
				{
					cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
					cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
					mnTracked++;
				}
			}
		}

	}

	cv::Mat imWithInfo;
	DrawTextInfo(im,state, imWithInfo);

	return imWithInfo;
}

void FramePublisher::PublishFrame()
{
	cv::Mat im = DrawFrame();
	cv_bridge::CvImage rosImage;
	rosImage.image = im.clone();
	rosImage.header.stamp = ros::Time::now();
	rosImage.encoding = "bgr8";

	mImagePub.publish(rosImage.toImageMsg());
	

	ros::spinOnce();
}

void FramePublisher::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{   
	stringstream s;
	slam_pub=mNH.advertise<std_msgs::Int8>("ORB_SLAM2/Failure",10);
	if(nState==Tracking::NO_IMAGES_YET)
		s << "WAITING FOR IMAGES. (Topic: /camera/image_raw)";
	else if(nState==Tracking::NOT_INITIALIZED)
		s << " TRYING TO INITIALIZE ";
	else if(nState==Tracking::OK)
	{
		s << " TRACKING ";
		int nKFs = mpMap->KeyFramesInMap();
		int nMPs = mpMap->MapPointsInMap();
		s << " - KFs: " << nKFs << " , MPs: " << nMPs << " , Tracked: " << mnTracked;
	  
		FramePublisher::ret_state(); //Publish tracked features
		 //Publish map points
		

	}
	else if(nState==Tracking::LOST)
	{
		s << " TRACK LOST. TRYING TO RELOCALIZE ";
	}
	else if(nState==Tracking::SYSTEM_NOT_READY)
	{
		s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
	}

	int baseline=0;
	cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

	imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
	im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
	imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
	cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FramePublisher::Update(Tracking *pTracker)
{
	boost::mutex::scoped_lock lock(mMutex);
	pTracker->mImGray.copyTo(mIm);
	mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
	mvpMatchedMapPoints=pTracker->mCurrentFrame.mvpMapPoints;
	mvbOutliers = pTracker->mCurrentFrame.mvbOutlier;

	if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
	{
		mvIniKeys=pTracker->mInitialFrame.mvKeys;
		mvIniMatches=pTracker->mvIniMatches;
	}
	
	mState=static_cast<int>(pTracker->mLastProcessedState);
	FramePublisher::slam_info();
	mbUpdated=true;
}

void FramePublisher::ret_state()
{   tracking_pub=mNH.advertise<std_msgs::Int16>("ORB_SLAM2/Tracked",10);
	tracking_pub.publish(FramePublisher::mnTracked);


}
void FramePublisher::slam_info()
{
	if (mState==Tracking::LOST)
		{
			slam_pub.publish(0);
			
		}

	else if(mState==Tracking::OK)
	{  
		slam_pub.publish(1);

	}



}
	

	
	


} //namespace ORB_SLAM2