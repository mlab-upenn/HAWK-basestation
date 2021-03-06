/*
pgdownsample: file for downsampling the point cloud before it is provided to the renderer.

Author: Paul Gurniak (pgurniak@gmail.com)

Date created: April 7, 2012

Note that all files for this project can be found in our Git repository:
https://github.com/mlab/HAWK-basestation

*/

#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr cloud_filtered(new sensor_msgs::PointCloud2 ());
pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
//pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB> ());

float fx_d = 5.9421e+02;
float fy_d = 5.9421e+02;
float cx_d = 3.393078e+02;
float cy_d = 2.428e+02;

void copyInColorData(Keyframe * kA)
{
  uint8_t * rgbBuf = (uint8_t *)kA->im->imageData;
  PointColor * points = (PointColor *)&(kA->points[0]);
  
  for(uint32_t i = 0; i < kA->numberOf3DPoints; i++) {
    int color_x = lroundf(points[i].x * fx_d / points[i].z + cx_d);
    int color_y = lroundf(points[i].y * fy_d / points[i].z + cy_d);
    int color_idx = 640*color_y + color_x;

    if(color_x >= 0 && color_x < 640 && color_y >= 0 && color_y < 480) { 
      points[i].r = rgbBuf[color_idx];
      points[i].g = rgbBuf[color_idx+1];
      points[i].b = rgbBuf[color_idx+2];
    }   

  }
}

void downsampleCloud(Keyframe * kA)
{

  
  cloud->fields.resize(4);
  cloud->fields[0].name = "x";
  cloud->fields[0].offset = 0;
  cloud->fields[0].datatype = 7;
  cloud->fields[0].count = 1;
  cloud->fields[1].name = "y";
  cloud->fields[1].offset = 4;
  cloud->fields[1].datatype = 7;
  cloud->fields[1].count = 1;
  cloud->fields[2].name = "z";
  cloud->fields[2].offset = 8;
  cloud->fields[2].datatype = 7;
  cloud->fields[2].count = 1;
  cloud->fields[3].name = "no-op";
  cloud->fields[3].offset = 12;
  cloud->fields[3].datatype = 6;
  cloud->fields[3].count = 1;
  
  cloud->height = 480;
  cloud->width = 640;
  cloud->is_dense = 1;
  cloud->is_bigendian = 1;
  cloud->point_step = 4*sizeof(float);
  cloud->row_step = 4*sizeof(float)*640;
  cloud->data.resize(640*480*4*sizeof(float));
  memcpy((uint8_t *)&(cloud->data[0]), (uint8_t *)&(kA->points[0]), 640*480*4*sizeof(float));

  sor.setInputCloud(cloud);
  sor.setDownsampleAllData(false);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*cloud_filtered);

  printf("Filtered cloud down to %d\n", cloud_filtered->height*cloud_filtered->width);
  
  memcpy((uint8_t *)&(kA->points[0]), &(cloud_filtered->data[0]),
	 4*sizeof(float)*cloud_filtered->height*cloud_filtered->width);
  
  kA->numberOf3DPoints = cloud_filtered->height*cloud_filtered->width;

  // Downsampling messes up the RGB data, so just copy it in here
  copyInColorData(kA);

}
