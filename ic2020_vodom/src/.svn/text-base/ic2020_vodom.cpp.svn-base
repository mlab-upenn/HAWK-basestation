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
 * <ic2020_vodom/src/ic2020_vodom.cpp>
 * 
 * revision 1.0
 */
 
 #include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <vector>

#include "highgui.h"
#include "cv.h"

#include "Constants.h"

#include <iostream>

#include <math.h> // for atan2f
#include "SURFHelper.h"
#include "VisualOdometry.h"
#include "Keyframe.h"

/////////////////////
// Fixed Constants //
/////////////////////

#define BW_IPL_PXL_BYTES 1
#define CLR_IPL_PXL_BYTES 3

const bool bw_not_color = false;
int IPL_PXL_BYTES = bw_not_color?BW_IPL_PXL_BYTES:CLR_IPL_PXL_BYTES;
std::string IPL_IMG_TYPE = bw_not_color?"mono8":"bgr8";
std::string SURF_IMG_TYPE = bw_not_color?"I":"BGR";

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

double tt;

// Get an OpenCV camera handle
Keyframe* kA = 0;
Keyframe* kB = 0;
Keyframe* newArrival = 0;
IplImage* view_im;

Vector3 initial_imu;

// Callback that recieves new frame
void optflowCallback(const ic2020_vodom::keyframe::ConstPtr& msg)
{
    newArrival = new Keyframe(msg);
}

// Thread block while polling for first frame
void BlockWhileWaitingForVideo()
{
    printf("Visual Odom waiting to make connection with Keyframe stream...\n");     
    while (ros::ok())
    {
        ros::spinOnce();
        if (newArrival != NULL)
            break;
    }
    printf("Visual Odom Keyframe streams found...\n");
}

void RotateNewArrivalIn()
{
    if (kA != NULL) { delete kA; }
    
    kA = kB;
    kB = newArrival;
    newArrival = 0;
    
    // Init rot and trans
    kB->rotation[0] = 1.0f;    kB->rotation[1] = 0.0f;    kB->rotation[2] = 0.0f;
    kB->rotation[3] = 0.0f;    kB->rotation[4] = 1.0f;    kB->rotation[5] = 0.0f;
    kB->rotation[6] = 0.0f;    kB->rotation[7] = 0.0f;    kB->rotation[8] = 1.0f;
    kB->translation[0] = 0.0f; kB->translation[1] = 0.0f; kB->translation[2] = 0.0f;
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "ic2020_vodom");
    ros::NodeHandle n;   

    ros::Subscriber surf_sub = n.subscribe("/surf/keyframes", 5, optflowCallback);
    ros::Publisher vodom_pub = n.advertise<ic2020_vodom::keyframe>("/vodom/keyframes", 100);    

    // Wait for video streams to be up
    BlockWhileWaitingForVideo();  

    // Create display images
	view_im = cvCreateImage( cvSize(2*newArrival->width, newArrival->height), 8, IPL_PXL_BYTES );

	cvNamedWindow("VisualOdom", CV_WINDOW_AUTOSIZE);

    // Main loop
    printf("Entering main loop\n");

    while (ros::ok())
    {
        char c = cvWaitKey(5);
        if (c == 'Q' || c == 'q')
            break; 

        // Get Images
        ros::spinOnce();

        // Check if new keyframe is available
        if (newArrival == NULL) { continue; }
        printf ("\33[2J");
        
        // Rotate in new data
        RotateNewArrivalIn();
        
        /**********************************
            Check we have two keyframes
        ***********************************/
        if (kA == 0 || kB == 0) { continue; }
        
        printf("Keyframe A: %i\n", kA->keyframe_num);
        printf("Keyframe B: %i\n", kB->keyframe_num);
        
        // COPY IMAGE DATA TO DOUBLE SIZE IMAGE
        cvSetImageROI( view_im, cvRect(0, 0, kB->im->width, kB->im->height));
        cvCopy( kB->im, view_im );
        cvSetImageROI( view_im, cvRect(kB->im->width, 0, kA->im->width, kA->im->height));
        cvCopy( kA->im, view_im );
        cvResetImageROI( view_im );

        // DRAW RED CIRCLES ON FEATURES
        for (unsigned int i = 0; i < kB->features.size(); i++) {
            cvCircle(view_im, cvPoint(cvRound(kB->features[i].point2D[0]),
                     cvRound(kB->features[i].point2D[1])), 3.0f, colors[0], 2, 8);
        }
        for (unsigned int i = 0; i < kA->features.size(); i++) {
            cvCircle(view_im, cvPoint(cvRound(kA->features[i].point2D[0]) + kB->im->width, 
                     cvRound(kA->features[i].point2D[1])), 3.0f, colors[0], 2, 8);              
        }
        for (unsigned int i = 0; i < kB->numCorn1; i++) {
            cvCircle(view_im, cvPoint(cvRound(kB->corn1[i].point2D[0]),
                     cvRound(kB->corn1[i].point2D[1])), 3.0f, colors[1], 1, 8);
        }
        for (unsigned int i = 0; i < kA->numCorn2; i++) {
            cvCircle(view_im, cvPoint(cvRound(kA->corn2[i].point2D[0]) + kB->im->width, 
                     cvRound(kA->corn2[i].point2D[1])), 3.0f, colors[1], 1, 8);              
        }
        
        /**********************************
          Initial RANSAC w SURF and STCorn
        ***********************************/
        
        // GET SURF PAIRS
        tt = (double)cvGetTickCount();
        std::vector<unsigned int> pairs;
        SURFHelper::findSURFPairs(&kA->descBuffer, &kB->descBuffer, pairs);
        tt = (double)cvGetTickCount() - tt;
        //printf( "SURF Match Time = %gms\n", tt/(cvGetTickFrequency()*1000.));        
        printf( "Found %i SURF Matches \n", pairs.size()/2);
        
        // RANSAC
        std::vector<unsigned int> filtered_surf_pairs;
        std::vector<unsigned int> filtered_corn_pairs;
        tt = (double)cvGetTickCount();
        if (kA->numCorn2 == kB->numCorn1) {
            if (!VisualOdometry::RANSAC6DFast(&kA->features, &kB->features, &pairs, &filtered_surf_pairs,
                                          &kA->corn2[0], &kB->corn1[0], &kB->status[0], kB->numCorn1, &filtered_corn_pairs,
                                          kB->im->width, kB->im->height, 10, 10, 1)) 
            //if (!VisualOdometry::RANSAC6D(&kA->features, &kB->features, &pairs, &filtered_surf_pairs,
            //                              &kA->corn2[0], &kB->corn1[0], &kB->status[0], kB->numCorn1, &filtered_corn_pairs)) 
            //if (!VisualOdometry::RANSAC6DReproj(&kA->features, &kB->features, &pairs, &filtered_surf_pairs))
            {
                printf("RANSAC MATCHES FEATURE # AREN'T EQUAL OR LESS THAN 7 FEATURES \n");
                continue;
            }
        } else {
            printf("WTF KEYFRAME A's FORWARD ST FEATURES != KEYFRAME B's BACK ST FEATURES \n");
        }
        tt = (double)cvGetTickCount() - tt;
        printf( "RANSAC Time = %gms\n", tt/(cvGetTickFrequency()*1000.));        

        // Create index links from B to A
        std::vector<int> revReferenceA;
        for(unsigned int i = 0; i < (unsigned int)kA->features.size(); i++) { revReferenceA.push_back(-1); }
        for(unsigned int i = 0; i < (unsigned int)filtered_surf_pairs.size()/2; i++ )
        {
            int a = filtered_surf_pairs[2*i+0];
            int b = filtered_surf_pairs[2*i+1]; 
            kB->surfMatches[b] = a;
            revReferenceA[a] = b;
        } 
     
        // Remove Useless SURF Features
        std::vector<feature> tfeatures;
        std::vector<surfdesc> tdescBuffer;
        std::vector<int> tsurfMatches;
        for (unsigned int i = 0; i < kA->features.size(); i++) {
            if (revReferenceA[i] >= 0) { // is being matched in the next frame
                tfeatures.push_back(kA->features[i]);
                tdescBuffer.push_back(kA->descBuffer[i]);
                tsurfMatches.push_back(kA->surfMatches[i]);
                kB->surfMatches[revReferenceA[i]] = tfeatures.size() - 1;
            }
            else if (kA->surfMatches[i] >= 0) { // has a match in the previous frame
                tfeatures.push_back(kA->features[i]);
                tdescBuffer.push_back(kA->descBuffer[i]);
                tsurfMatches.push_back(kA->surfMatches[i]);
            }
        }
        kA->features = tfeatures;
        kA->descBuffer = tdescBuffer;
        kA->surfMatches = tsurfMatches;
     
        // CREATE VECTOR OF MATCHES
        std::vector<feature> matchesA2;
        std::vector<feature> matchesB2;
        // ADD IN SURF MATCHES
        for(unsigned int i = 0; i < (unsigned int)filtered_surf_pairs.size()/2; i++ )
        {
            //int a = filtered_surf_pairs[2*i+0];
            int b = filtered_surf_pairs[2*i+1]; 
            int a = kB->surfMatches[b];
            matchesA2.push_back(kA->features[a]);
            matchesB2.push_back(kB->features[b]);
        } 
     
        // ADD IN CORNER MATCHES
        for(unsigned int i = 0; i < kB->numCorn1; i++ )
        {
            if (filtered_corn_pairs[i] > 0) {
                matchesA2.push_back(kA->corn2[i]);
                matchesB2.push_back(kB->corn1[i]);
            } else {
                kB->status[i] = 0;
            }
        }
        
        // Print Green Circles Over RANSAC Points
        for (unsigned int i = 0; i < matchesA2.size(); i++) {
            float lx = matchesB2[i].point2D[0];
            float ly = matchesB2[i].point2D[1];
            float lx2 = matchesA2[i].point2D[0];
            float ly2 = matchesA2[i].point2D[1];
            cvCircle(view_im,cvPoint(cvRound(lx), cvRound(ly)), 3.0f, colors[3], 2, 8);
            cvCircle(view_im, cvPoint(cvRound(lx2) + kA->im->width, cvRound(ly2)), 3.0f, colors[3], 2, 8);  
            cvLine(view_im, cvPoint(cvRound(lx), cvRound(ly)), cvPoint(cvRound(lx2), cvRound(ly2)), colors[3] );
        }

        // Least Squares
        double rdata[9];
        double translation[3];
        
        VisualOdometry::ArunLeastSquares(&matchesA2, &matchesB2, rdata, translation);      
        
        for (unsigned int i = 0; i < 9; i++) { kB->rotation[i] = rdata[i]; }
        for (unsigned int i = 0; i < 3; i++) { kB->translation[i] = translation[i]; }
        
        /*********************
            ICP
        **********************/
        // Setup Images and Contours
        
        // Call ICP
        /*double rdata2[9];
        double translation2[3];
        VisualOdometry::ICPKeyframes(kA, kB, rdata, translation, rdata2, translation2);
        for (unsigned int i = 0; i < 9; i++) { kB->rotation[i] = rdata2[i]; }
        for (unsigned int i = 0; i < 3; i++) { kB->translation[i] = translation2[i];}*/

        /*********************
            Publish Frame
        **********************/        
        
        // Print Rotation and Translation
        double pitch = atan2(-rdata[6], sqrt(pow(rdata[0],2.0)+pow(rdata[3],2.0)));
        double yaw = atan2(rdata[3]/cos(pitch), rdata[0]/cos(pitch));
        double roll = atan2(rdata[7]/cos(pitch), rdata[8]/cos(pitch));
        printf("pit yaw rol: %f %f %f\n",pitch,yaw,roll);
        printf("translation: %f %f %f\n",translation[0],translation[1],translation[2]); 
        
        /*double pitch2 = atan2(-rdata2[6], sqrt(pow(rdata2[0],2.0)+pow(rdata2[3],2.0)));
        double yaw2 = atan2(rdata2[3]/cos(pitch), rdata2[0]/cos(pitch));
        double roll2 = atan2(rdata2[7]/cos(pitch), rdata2[8]/cos(pitch));
        printf("icp pit yaw rol: %f %f %f\n",pitch2,yaw2,roll2);
        printf("icp translation: %f %f %f\n",translation2[0],translation2[1],translation2[2]); */

        // Publish Point Clouds
        printf("publishing\n");
        kA->PublishKeyframe(&vodom_pub);
        printf("done publishing\n");

        // Show stereo image
        cvShowImage("VisualOdom", view_im);
    }

    return 0;
}

