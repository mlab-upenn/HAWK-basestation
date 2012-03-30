/************************************************************************
 * 
 * This source code is part of the IC2020 SLAM System
 * 
 * IC2020 Copyright (C) 2011
 * Sean Anderson
 * Kirk MacTavish
 * 
 * IC2020 is licenced under the Creative Commons License
 * Attribution-NonCommercial-ShareAlike 2.5 Canada
 * (CC BY-NC-SA 2.5).
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *  
 *   - Noncommercial. You may not use this work for commercial purposes.
 *  
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * 
 ************************************************************************/
 
/* 
 * <ic2020_surf/src/ic2020_surf.cpp>
 * 
 * revision 1.0
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

// THESE ARE GPU SURF LIBRARIES
// THESE ARE NOT SIMPLE TO GET RUNNING,
// AND QUALITY VARIES WITH NVIDIA CARD TYPE
// http://homes.esat.kuleuven.be/~ncorneli/gpusurf/
#include "window.hpp"
#include "surfobject.hpp"

#include <string>
#include <vector>

#include "highgui.h"
#include "cv.h"

#include <iostream>

#include <math.h> // for atan2f

#include "ic2020_vodom/keyframe.h"
#include "Keyframe.h"
#include "SURFHelper.h"

#define BW_IPL_PXL_BYTES 1
#define CLR_IPL_PXL_BYTES 3
#define MIN_STEREO_DIST 0.1f

const bool bw_not_color = false;
int IPL_PXL_BYTES = bw_not_color?BW_IPL_PXL_BYTES:CLR_IPL_PXL_BYTES;
string IPL_IMG_TYPE = bw_not_color?"mono8":"bgr8";
string SURF_IMG_TYPE = bw_not_color?"I":"BGR";

static CvScalar colors[] = {
	{{0,0,255}},
	{{0,128,255}},
	{{0,255,255}},
	{{0,255,0}},
	{{255,128,0}},
	{{255,255,0}},
	{{255,0,0}},
	{{255,0,255}},
	{{255,255,255}}
};

// CUDA SURF Objects
ncglWindow glw;
SurfObject* surfer;

// CUDA SURF Params
bool upsurf     = false;
float surf_threshold = 0.04f; // 0.05 good... 0.01 is alot
//float stereo_row_thresh = 10.0f;
    
FeatureData rgb_fd;
double tt;

// Get an OpenCV camera handle
Keyframe* kA = 0;
Keyframe* newArrival = 0;
IplImage* view_im;

ros::Publisher surf_pub;

void optflowCallback(const ic2020_vodom::keyframe::ConstPtr& msg)
{
    newArrival = new Keyframe(msg);
}

void BlockWhileWaitingForVideo()
{
    printf("SURF waiting to make connection with kinect video stream...\n");     
    while (1)
    {
        ros::spinOnce();
        if (newArrival != NULL)
            break;
        char c = cvWaitKey(5);
        if (c == 'Q' || c == 'q')
            break; 

    }
    printf("SURF video streams found...\n"); 
}

void RotateNewArrivalIn()
{
    if (kA != NULL) { delete kA; }
    kA = newArrival;
    newArrival = NULL;
}

int main(int argc, char **argv)
{

    // Initialize ROS
    ros::init(argc, argv, "ic2020_surf");
    ros::NodeHandle n;   

    ros::Subscriber flow_sub = n.subscribe("/flow/keyframes", 5, optflowCallback);
    surf_pub = n.advertise<ic2020_vodom::keyframe>("/surf/keyframes", 5);    
       
        
    // Wait for video streams to be up
    BlockWhileWaitingForVideo();  

    // Create display images
	view_im = cvCreateImage( cvSize(newArrival->width, newArrival->height), 8, IPL_PXL_BYTES );

	cvNamedWindow("SURF", CV_WINDOW_AUTOSIZE);

    // Initialize SURFer
	fprintf(stderr,"starting surfer...");
	glw.init("GPU-surf",256,256,false);
	surfer = new SurfObject;
    surfer->setWindow(&glw);
	surfer->setVerbose(true);
	surfer->setUpSurf(upsurf);
	surfer->setThreshold(surf_threshold);

	fprintf(stderr,"done creating surfer...\n");

    // Main loop
    printf("Entering SURF Main Loop\n");
    while (ros::ok())
    {
        char c = cvWaitKey(5);
        if (c == 'Q' || c == 'q')
            break;

        // Get Images
        ros::spinOnce();

        // Check if new keyframe is available
        if (newArrival == NULL) { continue; }
        
        // Rotate in new data
        RotateNewArrivalIn();
        
        // *****************************************
        // Perform operations on new keyframe
        // *****************************************
        
        // Get SURF Descriptors and 2D Coords
        surfer->loadMemoryImage(SURF_IMG_TYPE.c_str(), kA->im->width, kA->im->height, (unsigned char*)kA->im->imageData);         
        surfer->run(&rgb_fd);   
        
        printf("found %i SURF features\n", rgb_fd.nrf);
        
        // Create SURF Features
        float* rgb_points2D = new float[rgb_fd.nrf*2];
	    CvMat rgb_points2DMat = cvMat(rgb_fd.nrf, 1, CV_32FC2, &rgb_points2D[0]); //64?
        for ( unsigned int row = 0; row < rgb_fd.nrf; row++ )
        {
            CV_MAT_ELEM(rgb_points2DMat, pos2D, row, 0).f1 = rgb_fd.posdata[row*4];
            CV_MAT_ELEM(rgb_points2DMat, pos2D, row, 0).f2 = rgb_fd.posdata[row*4+1];
        }
	    kA->features.clear();
	    kA->descBuffer.clear();
	    kA->surfMatches.clear();
        for (unsigned int a = 0; a < rgb_fd.nrf; a++ )
        {
            feature new_feature;
            
            new_feature.point2D[0] = CV_MAT_ELEM(rgb_points2DMat, pos2D, a, 0).f1;
            new_feature.point2D[1] = CV_MAT_ELEM(rgb_points2DMat, pos2D, a, 0).f2;
		    PointColor tempP = kA->points[cvRound(new_feature.point2D[1])*kA->point_step + 
		                                  cvRound(new_feature.point2D[0])];
		    new_feature.point3D[0] = tempP.x;
		    new_feature.point3D[1] = tempP.y;
		    new_feature.point3D[2] = tempP.z;

		    if (new_feature.point3D[2] > 0.25f) {
                surfdesc new_desc;
                memcpy(&new_desc.desc[0], &rgb_fd.desc_data[a*64], 64*sizeof(float));
                kA->descBuffer.push_back(new_desc);
                kA->features.push_back(new_feature);
                kA->surfMatches.push_back(-1);                
            }
	    }
	    delete [] rgb_points2D;
        
        // COPY IMAGE DATA
        cvCopy( kA->im, view_im );

        // DRAW RED CIRCLES ON FEATURES
        for (unsigned int i = 0; i < kA->features.size(); i++) {
            cvCircle(view_im, cvPoint(cvRound(kA->features[i].point2D[0]), 
                     cvRound(kA->features[i].point2D[1])), 3.0f, colors[0], 2, 8);              
        }
        
        // Publish Point Clouds
        kA->PublishKeyframe(&surf_pub);

        // Show surf image
        cvShowImage("SURF", view_im);
        
    }

    delete surfer;
    surfer = NULL;

    return 0;
}

