# ORB-SLAM2 - for ROS

Modified version of ORB_SLAM2 with full ROS inputs and ROS outputs of paths, pose and pointcloud. Find the original at https://github.com/raulmur/ORB_SLAM2

Only the monocular node has been tested and the Settings file provided is only for monocular. Please look at the original for settings for RGBD and Stereo nodes and how to run them.

Usage: rosrun ORB_SLAM2 orb_mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE