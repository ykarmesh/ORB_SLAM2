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

#ifndef FRAMEPUBLISHER_H
#define FRAMEPUBLISHER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread.hpp>
#define FRAME_GRID_ROWS_ 6
#define FRAME_GRID_COLS_ 8


namespace ORB_SLAM2
{

class Tracking;


class FramePublisher
{
public:
	FramePublisher(Map* pMap);	

	void Update(Tracking *pTracker);

	void Refresh();

	void SetMap(Map* pMap);
	
protected:
	const char* MAP_FRAME_ID = "/world";
        const char* CAMERA_FRAME_ID = "/camera_rgb_optical_frame";
    
	int mnTracked;

	cv::Mat DrawFrame();

	void PublishFrame();
	void PublishSLAMStatus();
	void PublishFeatureInfo();	

	void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);


	cv::Mat mIm;
	vector<cv::KeyPoint> mvCurrentKeys;

	vector<bool> mvbOutliers;

	vector<MapPoint*> mvpMatchedMapPoints;

	vector<MapPoint*> mvpLocalMapPoints;
	
	vector<cv::KeyPoint> mvIniKeys;
	vector<int> mvIniMatches;

	ros::NodeHandle mNH;
	ros::Publisher mImagePub, mFramePointsPub, mSLAMStatusPub, mFeatureInfoPub;
	sensor_msgs::PointCloud2 mMapPointCloud;
	
	int mState;

	bool mbUpdated;
	std::vector<std::size_t> nGrid[FRAME_GRID_COLS_][FRAME_GRID_ROWS_];
        //std::vector<std::size_t> *nGrid;

	Map* mpMap;

	boost::mutex mMutex;
};

} //namespace ORB_SLAM2

#endif // FRAMEPUBLISHER_H
