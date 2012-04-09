/**
 * Simple wrappers around the PCL simple cloud visualizer
 */

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
