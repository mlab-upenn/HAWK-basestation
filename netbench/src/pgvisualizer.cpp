/*
Netbench: program for commmunication with the Intel Atom and Microsoft Kinect onboard the quadrotor.  This program receives data from the Intel Atom, decompresses it, and publishes to predefined topics in the ROS system.

Author: Paul Gurniak (pgurniak@gmail.com)

Date created: March 16, 2012

Note that all files for this project can be found in our Git repository:
https://github.com/mlab/HAWK-basestation

*/

// This file is essentially test code: it allows us to visualize the constructed point cloud
// before it is sent to the SLAM system.

#ifndef INCONCE
#include "pgcommon.h"
#include "pgdecompress.h"
#define INCONCE
#endif

#include "pgvisualizer.h"
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ros/conversions.h>

pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

void show_cloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  
  viewer.showCloud(cloud);

}
