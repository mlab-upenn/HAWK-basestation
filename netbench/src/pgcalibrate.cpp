/**
 * Contains various functions to convert standard Kinect data
 * (RGB array and depth array) into a point cloud.  This
 * includes image registration and depth to real world
 * value conversion.
 */

#ifndef INCONCE
#include "pgcommon.h"
#include "pgdecompress.h"
#define INCONCE
#endif

// Constants:
#define NO_SAMPLE_VALUE 0x07FF
#define SHADOW_VALUE 0x0000
#define K_FOCAL_LENGTH 525

// If you want to rotate (flip) the image across the
// coordinate axes, uncomment the following defines
#define FLIP_X_AXIS
#define FLIP_Y_AXIS

typedef struct {
  uint16_t u;
  uint16_t v;
} POINT_REMAP;

typedef union {
  struct {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

POINT_REMAP * rgb_remap = NULL;
POINT_REMAP * depth_remap = NULL;

void makePointCloud(const uint8_t * rgb_buf, const float * depth_buf,
		    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg)
{
  float centerX, centerY;
  cloud_msg->is_dense = false;
  cloud_msg->height = 640;
  cloud_msg->width = 480;
  centerX = (cloud_msg->width >> 1) - 0.5f;
  centerY = (cloud_msg->height >> 1) - 0.5f;

  cloud_msg->points.resize(cloud_msg->height * cloud_msg->width);

  int color_idx = 0, depth_idx = 0;
  int rgb_step = 640*3;
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud_msg->begin();

  for(int v = 0; v < (int)cloud_msg->height, v++, color_idx += 0) {
    for(int u = 0; u < (int)cloud_msg->width; u++, color_idx += 3, depth_idx++, pt_iter++) {
      pcl::PointXYZRGB& pt = *pt_iter;
      pcl::PointXYZRGB temp_pt;
      float Z = depth_buf[depth_idx];
      
      if(Z != Z || Z == 0.0f) {
	pt.x = pt.y = pt.z = bad_point;
	pt.rgb = bad_point;
      } else {

	pt.x = (u - cx_d) * Z / fx_d;
	pt.y = (v - cy_d) * Z / fy_d;
	pt.z = Z;

	// Apply rotation and translation
	temp_pt.x = pt.x*rotate[0] + pt.y*rotate[1] + pt.z*rotate[2] + TRAN_X;
        temp_pt.y = pt.x*rotate[3] + pt.y*rotate[4] + pt.z*rotate[5] + TRAN_Y;
        temp_pt.z = pt.x*rotate[6] + pt.y*rotate[7] + pt.z*rotate[8] + TRAN_Z;

        uint16_t color_x = (uint16_t)lroundf(temp_pt.x * fx_rgb / temp_pt.z + cx_rgb);
        uint16_t color_y = (uint16_t)lroundf(temp_pt.y * fy_rgb / temp_pt.z + cy_rgb);

	// Need to check to see whether reprojected point is still within the bitmap
	if(color_x >= K_RGB_WIDTH || color_y >= K_RGB_HEIGHT) {
          pt.x = pt.y = pt.z = bad_point;
          pt.rgb = bad_point;
          continue;
        }

        color_idx = rgb_step*color_y + color_x*K_RGB_BYTES;

        RGBValue color;
        color.Red = rgb_buf[color_idx];
        color.Green = rgb_buf[color_idx + 1];
        color.Blue = rgb_buf[color_idx + 2];
        color.Alpha = 0;

	// Invalidate RGB points taht are now empty due to undistortion
	if(color.long_value == 0) {
          pt.x = pt.y = pt.z = bad_point;
          pt.rgb = bad_point;
        } else {
          pt.rgb = color.float_value;
        }

      }
    }
  }

}


void flip_map(POINT_REMAP * remap)
{
  
  for(int j = 0; j < 480; j++) {
    for(int i = 0; i < 640; i++) {
      int u = (int)remap[640*j + i].u;
      int v = (int)remap[640*j + i].v;
      //      u = 640 - u;
      v = 480 - v;
      remap[640*j + i].u = (uint16_t)u;
      remap[640*j + i].v = (uint16_t)v;
    }
  }

}

void compute_rgb_map( void )
{
  if(rgb_remap != NULL) {
    return;
  }
  rgb_remap = (POINT_REMAP *)malloc(640*480*sizeof(POINT_REMAP));

  // For each point (i, j) in the raw RGB frame, calculate its
  // destination (ii, jj) in the undistorted image, and store
  // the result as an entry (i, j) in map_dest.
  
  int i, j, ii, jj;
  float xsq, x, ysq, y, xx, yy, uu, vv, u, v, rsq;
  
  for(j = 0; j < 480; j++) {
    for(i = 0; i < 640; i++) {
      // Apply Brown's (un)distortion model
      u = i - cx_rgb;
      v = j - cy_rgb;
      x = u/fx_rgb;
      y = v/fy_rgb;
      xsq = x*x;
      ysq = y*y;
      rsq = xsq + ysq;
      xx = x*(1+k1_rgb*rsq + k2_rgb*rsq*rsq + k3_rgb*rsq*rsq*rsq) +
        2*p2_rgb*x*y + p1_rgb*(rsq+2*xsq);
      yy = y*(1+k1_rgb*rsq + k2_rgb*rsq*rsq + k3_rgb*rsq*rsq*rsq) +
        p2_rgb*(rsq + 2*ysq) + 2*p1_rgb*x*y;
      uu = xx*fx_rgb;
      ii = uu + cx_rgb + 0.5;
      vv = yy*fy_rgb;
      jj = vv + cy_rgb + 0.5;
      rgb_remap[j*K_RGB_WIDTH + i].u = ii;
      rgb_remap[j*K_RGB_WIDTH + i].v = jj;      
    }
  }

  flip_map(rgb_remap);

}

void compute_depth_map( void )
{
  if(depth_remap != NULL) {
    return;
  }
  depth_remap = (POINT_REMAP *)malloc(640*480*sizeof(POINT_REMAP));

  // For each point (i, j) in the raw RGB frame, calculate its
  // destination (ii, jj) in the undistorted image, and store
  // the result as an entry (i, j) in map_dest.
  
  int i, j, ii, jj;
  float xsq, x, ysq, y, xx, yy, uu, vv, u, v, rsq;
  
  for(j = 0; j < 480; j++) {
    for(i = 0; i < 640; i++) {
      // Apply Brown's (un)distortion model
      u = i - cx_d;
      v = j - cy_d;
      x = u/fx_d;
      y = v/fy_d;
      xsq = x*x;
      ysq = y*y;
      rsq = xsq + ysq;
      xx = x*(1+k1_d*rsq + k2_d*rsq*rsq + k3_d*rsq*rsq*rsq) +
        2*p2_d*x*y + p1_d*(rsq+2*xsq);
      yy = y*(1+k1_d*rsq + k2_d*rsq*rsq + k3_d*rsq*rsq*rsq) +
        p2_d*(rsq + 2*ysq) + 2*p1_d*x*y;
      uu = xx*fx_d;
      ii = uu + cx_d + 0.5;
      vv = yy*fy_d;
      jj = vv + cy_d + 0.5;
      depth_remap[j*K_DEPTH_WIDTH + i].u = ii;
      depth_remap[j*K_DEPTH_WIDTH + i].v = jj;      
    }
  }

  flip_map(depth_remap);

}

// With the precalculated maps above, these two functions became 
// a lot simpler - the remapping process has been simplified to 
// an array dereference for each pixel.
void undistort_rgb(const uint8_t * rgb_buf, uint8_t * dest_buf)
{

  int i, j, u, v, height, width;
  height = K_RGB_HEIGHT;
  width = K_RGB_WIDTH;
  bzero(dest_buf, height*width*K_RGB_BYTES);
  for(j = 0; j < height; j++) {
    for(i = 0; i < width; i++) {
      u = rgb_remap[j*width+i].u;
      v = rgb_remap[j*width+i].v;
      // Check remapped point for validity
      if(u >= 0 && v >= 0 && u < width && v < height) {
        dest_buf[(j*width+i)*3+0] = rgb_buf[(v*width+u)*3+0];
        dest_buf[(j*width+i)*3+1] = rgb_buf[(v*width+u)*3+1];
        dest_buf[(j*width+i)*3+2] = rgb_buf[(v*width+u)*3+2];
      }
    }
  }
}


// With the precalculated maps above, these two functions became
// a lot simpler - the remapping process has been simplified to
// an array dereference for each pixel.
void undistort_depth(const uint16_t * depth_buf, uint16_t * dest_buf)
{

  int i, j, u, v, height, width;
  height = K_DEPTH_HEIGHT;
  width = K_DEPTH_WIDTH;
  bzero(dest_buf, height*width*sizeof(uint16_t));
  for(j = 0; j < height; j++) {
    for(i = 0; i < width; i++) {
      u = depth_remap[j*width+i].u;
      v = depth_remap[j*width+i].v;
      // Check remapped point for validity 
      if(u >= 0 && v >= 0 && u < width && v < height) {
        dest_buf[(j*width+i)] = depth_buf[(v*width+u)];
      }
    }
  }
}

