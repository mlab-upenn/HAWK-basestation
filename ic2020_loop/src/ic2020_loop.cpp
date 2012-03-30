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
 * <ic2020_loop/src/ic2020_loop.cpp>
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
#include <map>

#include <math.h>
#include "SURFHelper.h"
#include "VisualOdometry.h"

#include "ic2020_vodom/keyframe.h"
#include "Keyframe.h"
#include "CornerHelper.h"

#include "ic2020_toro/newedge.h"
#include "ic2020_toro/loopnotice.h"

/////////////////////
// Fixed Constants //
/////////////////////

#define BW_IPL_PXL_BYTES 1
#define CLR_IPL_PXL_BYTES 3

#define SEARCH_MATCHES 2
#define SURF_SIZE 64

///////////////////////////
// Adjustable Parameters //
///////////////////////////

#define VOTE_MATCHES 4 // # of frames to try to loop close against
#define LOOP_DETEC_MIN_QUALITY 0.3 // Required quality for a loop detection

const bool bw_not_color = false;
int IPL_PXL_BYTES = bw_not_color?BW_IPL_PXL_BYTES:CLR_IPL_PXL_BYTES;
std::string IPL_IMG_TYPE = bw_not_color?"mono8":"bgr8";
std::string SURF_IMG_TYPE = bw_not_color?"I":"BGR";

ros::Publisher edge_pub;
ros::Publisher loop_notice_pub;

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

struct Link {
	Link(int frame_, int feature_){frame = frame_; feature = feature_;}
	int frame;
	int feature;
};

// Global stuff
std::vector<surfdesc> glblFeatures;
std::vector<std::vector<Link> > glblChains;
std::vector<std::set<int> > glblFrameLinks;
std::vector<std::set<int> > glblEdges;
std::list<std::pair<int,int> > closureQueue;
std::vector<std::set<int> > closureChecked;

double tt;
CornerHelper cornHelp;
std::vector<Keyframe*> keyframes;

// Get an OpenCV camera handle
Keyframe* newArrival = 0;
IplImage* view_im;
IplImage* imgA = NULL;
IplImage* imgB = NULL;

void keyframeCallback(const ic2020_vodom::keyframe::ConstPtr& msg)
{
    newArrival = new Keyframe(msg);
    keyframes.push_back(newArrival);
}

void BlockWhileWaitingForVideo()
{
    printf("ic2020 Loop waiting to make connection with Keyframe stream...\n");     
    while (ros::ok())
    {
        ros::spinOnce();
        if (newArrival != NULL)
            break;
    }
    printf("ic2020 Loop Keyframe streams found...\n");
}

void publishEdge(int prime, int observed, float* rotation, float* translation) {

	ic2020_toro::newedge msg;
    int num_bytes;
	
	// Prime Keyframe Number
    msg.prime_keyframe = keyframes[prime]->keyframe_num;
	
	// Obs Keyframe Number
	if (observed >= 0) {
		glblEdges[prime].insert(observed);
		glblEdges[observed].insert(prime);
	    msg.obs_keyframe = keyframes[observed]->keyframe_num;
	} else {
        msg.obs_keyframe = -1;
	}

    // Rotation and Translation Data    
    num_bytes = 9*sizeof(float);
    msg.rot.resize(num_bytes);
    float* temprot = reinterpret_cast<float*>(&msg.rot[0]);
    memcpy(temprot, rotation, num_bytes);
    
    num_bytes = 3*sizeof(float);
    msg.trans.resize(num_bytes);
    float* temptrans = reinterpret_cast<float*>(&msg.trans[0]);
    memcpy(temptrans, translation, num_bytes);

    edge_pub.publish(msg);
    
    // Tell TORO to loop-close
    ic2020_toro::loopnotice msgl;
    msgl.close = 1;
    loop_notice_pub.publish(msgl);
}

void detectLoopClosure () 
{
    
    /////////////////
    // Ctrs & Copy //
    /////////////////
    
    // Update frame & feature counters
    int curFrameIndex = keyframes.size()-1;
    
    Keyframe *newKeyframe = keyframes.back();
    
    // Copy Links & Feature locations
    glblFrameLinks.push_back(std::set<int>());
    
    // Print update
    std::cout << "Added frame " << keyframes[curFrameIndex]->keyframe_num <<
    		" with " << newKeyframe->surfMatches.size() << " features.\n";
    
    ///////////////////
    // Add to Global //
    ///////////////////
    
    for (unsigned int i = 0; i < newKeyframe->surfMatches.size(); i++)
    {
    	// Initialize New Feature
    	/////////////////////////// OR IF YOU MISSED A KEYFRAME, AND ADD TO RANSAC Q? OR JUST RUN RANSAC ....
    	if (newKeyframe->surfMatches[i] < 0 || curFrameIndex == 0) {
    		// New feature counter for the tree size
    		
    		// Global information for new features/chains
			glblFeatures.push_back(newKeyframe->descBuffer[i]);
			glblChains.push_back(std::vector<Link>(1,Link(curFrameIndex, i)));
			
			// Link to new global feature index
			newKeyframe->surfMatches[i] = glblChains.size()-1;
			
		// Link Chained Features
    	} else {
    		// Link to global feature index (chain stored in old links)
    		newKeyframe->surfMatches[i] = keyframes[keyframes.size()-2]->surfMatches[newKeyframe->surfMatches[i]];
    		
    		// Connect frames
    		for (unsigned int j = 0; j < glblChains[newKeyframe->surfMatches[i]].size(); j++) {
    			glblFrameLinks[curFrameIndex].insert(glblChains[newKeyframe->surfMatches[i]][j].frame);
				glblFrameLinks[glblChains[newKeyframe->surfMatches[i]][j].frame].insert(curFrameIndex);
    		}
    		
    		// Add the frame to the chain
			glblChains[newKeyframe->surfMatches[i]].push_back(Link(curFrameIndex, i));
		}
    }
    
    std::cout << "Total chains: " << glblChains.size() << "\n";
    
    /////////////////
    // Loop Detect //
    /////////////////
    
    // Waiting until we have enough frames to loopclose
    if (curFrameIndex == 0) {
		std::cout << "First frame, no loop detection\n\n";
		return;
    }
	
	std::vector<std::vector<unsigned int> > ptpairs;
	SURFHelper::findNSURFPairsKD(&(newKeyframe->descBuffer), &glblFeatures, ptpairs, SEARCH_MATCHES);
	
	// Identify the most likely loop frame
	// Votes are cast to every frame in matched chains
	std::map<int,int> votes;
	std::cout << "Pairs: " << ptpairs.size() << "\n";
	// Loop through each feature in the new frame
	for (unsigned int pair_ind = 0; pair_ind < ptpairs.size(); pair_ind++) {
		for (unsigned int match_ind = 1; match_ind < ptpairs[pair_ind].size(); match_ind++) {
			unsigned int j = ptpairs[pair_ind][match_ind];
			
			// Ensure that the matched feature is not matching to its own chain
			if (curFrameIndex == glblChains[j].back().frame) {
				continue;
				//printf("Feature %i linked to its own chain %i in frame %i\n", i, j, glblChains[j].front().frame);
			}
			
			// Cast votes for each frame in the matched chain
			int chainSize = glblChains[j].size();
		
			for (int k = 0; k < chainSize; k++) {
				int frameInd = glblChains[j][k].frame;
				
				// Add the vote
				if (votes.find(frameInd) == votes.end()) {
					votes[frameInd] = 1;
				} else {
					votes[frameInd]++;
				}
			}
			
		}
	}
	
	// Print match stats
	if (votes.empty()) {
		std::cout << "No votes cast\n\n";
		return;
	}
	for (std::map<int,int>::iterator voteInd = votes.begin(); voteInd != votes.end(); voteInd++) {
		//printf("%i votes cast for frame %i.\n",
		//		voteInd->second, keyframes[voteInd->first]->keyframe_num);
	}
	
	// Sort by vote count
	std::multimap<int,int> vote_cnt;
	for (std::map<int,int>::iterator voteInd = votes.begin(); voteInd != votes.end(); voteInd++) {
		vote_cnt.insert(std::pair<int,int>(voteInd->second, voteInd->first));
	}
	
	// Find the top n unlinked frames
	std::set<int> vote_blacklist;
	int found_frames = 0;
	std::cout << "\n";
	for (std::map<int,int>::reverse_iterator cntInd = vote_cnt.rbegin(); cntInd != vote_cnt.rend(); cntInd++) {
		if (vote_blacklist.find(cntInd->second) == vote_blacklist.end()) {
			std::cout << "Check frame " << keyframes[cntInd->second]->keyframe_num
					<< ", votes: " << cntInd->first << "\n";
			
			closureQueue.push_front(std::pair<int,int>(curFrameIndex, cntInd->second));
			
			for (std::set<int>::iterator linkInd = glblFrameLinks[cntInd->second].begin();
					linkInd != glblFrameLinks[cntInd->second].end(); linkInd++) {
				vote_blacklist.insert(*linkInd);
			}
			if (++found_frames >= VOTE_MATCHES)
				break;
		}
	}
	
    std::cout << "\n";
    
    return;
}

void CheckForLoopAndPub(int iA, int iB)
{
	// DO WE REALLY NEED THIS CHECK IF WE HAVE THE ONE BELOW?
	if (glblEdges[iA].find(iB) != glblEdges[iA].end())
		return;
	
	if (closureChecked[iA].find(iB) != closureChecked[iA].end())
		return;
	
	printf("Checking %i vs %i, Q: %i\n", keyframes[iA]->keyframe_num,
			keyframes[iB]->keyframe_num, closureQueue.size());
	
	closureChecked[iA].insert(iB);
	closureChecked[iB].insert(iA);
	
	Keyframe* kA = keyframes[iA];
	Keyframe* kB = keyframes[iB];
	
printf("Loop error hunt 1\n");
	
    // COPY IMAGE DATA TO DOUBLE SIZE IMAGE
    cvSetImageROI( view_im, cvRect(0, 0, kB->im->width, kB->im->height));
    cvCopy( kB->im, view_im );
    cvSetImageROI( view_im, cvRect(kB->im->width, 0, kA->im->width, kA->im->height));
    cvCopy( kA->im, view_im );
    cvResetImageROI( view_im );

    // Get Corners
    std::vector<feature> cornA;
    std::vector<feature> cornB;
    std::vector<char> cornpairs;
    cornHelp.FindFeatures(kA, kB, &cornA, &cornB, &cornpairs);
printf("Loop error hunt 2\n");
    // DRAW RED CIRCLES ON FEATURES
    for (unsigned int i = 0; i < kB->features.size(); i++) {
        cvCircle(view_im, cvPoint(cvRound(kB->features[i].point2D[0]),
                 cvRound(kB->features[i].point2D[1])), 3.0f, colors[0], 2, 8);
    }
    for (unsigned int i = 0; i < kA->features.size(); i++) {
        cvCircle(view_im, cvPoint(cvRound(kA->features[i].point2D[0]) + kB->im->width, 
                 cvRound(kA->features[i].point2D[1])), 3.0f, colors[0], 2, 8);              
    }
    for (unsigned int i = 0; i < cornA.size(); i++) {
        cvCircle(view_im, cvPoint(cvRound(cornB[i].point2D[0]),
                                  cvRound(cornB[i].point2D[1])), 3.0f, colors[1], 1, 8);
        cvCircle(view_im, cvPoint(cvRound(cornA[i].point2D[0]) + kB->im->width, 
                                  cvRound(cornA[i].point2D[1])), 3.0f, colors[1], 1, 8);              
    }
    
    // GET SURF PAIRS
    tt = (double)cvGetTickCount();
    std::vector<unsigned int> pairs;
    SURFHelper::findSURFPairs(&kA->descBuffer, &kB->descBuffer, pairs);
    tt = (double)cvGetTickCount() - tt;
    //printf( "SURF Match Time = %gms\n", tt/(cvGetTickFrequency()*1000.));        
    //printf( "Found %i SURF Matches \n", pairs.size()/2);

printf("Loop error hunt 3\n");
    
    // RANSAC
    std::vector<unsigned int> filtered_surf_pairs;
    std::vector<unsigned int> filtered_corn_pairs;
    tt = (double)cvGetTickCount();
    if (!VisualOdometry::RANSAC6DFast(&kA->features, &kB->features, &pairs, &filtered_surf_pairs,
                                  &cornA[0], &cornB[0], &cornpairs[0], cornA.size(), &filtered_corn_pairs,
                                  kB->im->width, kB->im->height, 10, 10, 1)) 
    {
        printf("RANSAC MATCHES FEATURE # AREN'T EQUAL OR LESS THAN 7 FEATURES \n");
        return;
    }
    tt = (double)cvGetTickCount() - tt;
    //printf( "RANSAC Time = %gms\n", tt/(cvGetTickFrequency()*1000.));
 
 printf("Loop error hunt 4\n");
 
    // CREATE VECTOR OF MATCHES
    std::vector<feature> matchesA2;
    std::vector<feature> matchesB2;
    // ADD IN SURF MATCHES
    for(unsigned int i = 0; i < (unsigned int)filtered_surf_pairs.size()/2; i++ )
    {
        int a = filtered_surf_pairs[2*i+0];
        int b = filtered_surf_pairs[2*i+1]; 
        matchesA2.push_back(kA->features[a]);
        matchesB2.push_back(kB->features[b]);
    } 
 
    // ADD IN CORNER MATCHES
    for(unsigned int i = 0; i < cornA.size(); i++ )
    {
        if (filtered_corn_pairs[i] > 0) {
            matchesA2.push_back(cornA[i]);
            matchesB2.push_back(cornB[i]);
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

    // Calculate Quality
    double total_feats = (double)(cornA.size()+pairs.size()/2);
    double quality_weight = (1 - pow(1.05, -total_feats));
    double quality = quality_weight*((double)matchesA2.size())/total_feats;
    printf( "RANSAC Quality!! = %f\n", quality);
    
    if (quality < LOOP_DETEC_MIN_QUALITY)
        return;
    printf("Keyframe %i IS LOOP CLOSED WITH Keyframe %i\n", kA->keyframe_num, kB->keyframe_num);
    
    
    // Least Squares
    double rdata[9];
    double translation[3];
    VisualOdometry::ArunLeastSquares(&matchesA2, &matchesB2, rdata, translation);      

//    for (unsigned int i = 0; i < 9; i++) { kB->rotation[i] = rdata[i]; }
//    for (unsigned int i = 0; i < 3; i++) { kB->translation[i] = translation[i]; }
    
    float rdataf[9];
    float translationf[9];
    for (unsigned int i = 0; i < 9; i++) { rdataf[i] = rdata[i]; }
    for (unsigned int i = 0; i < 3; i++) { translationf[i] = translation[i]; }
    
    // Print Rotation and Translation
    double pitch = atan2(-rdata[6], sqrt(pow(rdata[0],2.0)+pow(rdata[3],2.0)));
    double yaw = atan2(rdata[3]/cos(pitch), rdata[0]/cos(pitch));
    double roll = atan2(rdata[7]/cos(pitch), rdata[8]/cos(pitch));
    printf("pit yaw rol: %f %f %f\n",pitch,yaw,roll);
    printf("translation: %f %f %f\n",translation[0],translation[1],translation[2]); 

    // Show stereo image
    cvShowImage("LoopDetect", view_im);
    
    //////////////////
    // Publish Edge //
    //////////////////
    
    publishEdge(iB, iA, &rdataf[0], &translationf[0]);
    
    // THE CHAINING STUFF TOOK WAY TOO MUCH COMPUTATION TIME
    // FIND A BETTER WAY TO DO THIS LATER...
    
    /*
    // Add chains to queue if fresh loop closure
    for (std::set<int>::iterator i = glblFrameLinks[iA].begin(); i != glblFrameLinks[iA].end(); i++) {
    	if (*i != iB && closureChecked[iB].find(*i) == closureChecked[iB].end()) {
    		closureQueue.push_front(std::pair<int,int>(iB, *i));
    		printf("Adding KF %i vs KF %i\n", keyframes[iB]->keyframe_num, keyframes[*i]->keyframe_num);
    	}
    }
    for (std::set<int>::iterator i = glblFrameLinks[iB].begin(); i != glblFrameLinks[iB].end(); i++) {
    	if (*i != iA && closureChecked[iA].find(*i) == closureChecked[iA].end()) {
    		closureQueue.push_front(std::pair<int,int>(iA, *i));
    		printf("Adding KF %i vs KF %i\n", keyframes[iA]->keyframe_num, keyframes[*i]->keyframe_num);
    	}
    }
    */
	printf("\n");
    
    glblFrameLinks[iB].insert(iA);
    glblFrameLinks[iA].insert(iB);
}

void CheckForLoopAndPubQ() {

	if (closureQueue.empty())
		return;
	
	int iA = closureQueue.front().first;
	int iB = closureQueue.front().second;
	
	closureQueue.pop_front();
	
	CheckForLoopAndPub(iA, iB);
}

void publishNewFrame() {
	int keyframe_done = keyframes.size() - 1;
	
	Keyframe* kB = keyframes[keyframe_done];
    if (keyframes.size() < 2) {
    
        // Publish edges for the FIRST new node ONLY
    	publishEdge(0, -1, &kB->rotation[0], &kB->translation[0]);
		closureChecked.push_back(std::set<int>());
        return;
	}
	
	// Publish edges for the new nodes
	glblEdges.resize(keyframes.size());
	publishEdge(keyframe_done, keyframe_done-1, &kB->rotation[0], &kB->translation[0]);
	
	// NEED TO ADD EDGE TO RANSAC BLACKLIST....
	closureChecked.push_back(std::set<int>());
	closureChecked[keyframe_done].insert(keyframe_done-1);
	closureChecked[keyframe_done-1].insert(keyframe_done);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "ic2020_loop");
    ros::NodeHandle n;   

    ros::Subscriber surf_sub = n.subscribe("/vodom/keyframes", 5, keyframeCallback);
    edge_pub = n.advertise<ic2020_toro::newedge>("/loop/edges", 1000);
    loop_notice_pub = n.advertise<ic2020_toro::loopnotice>("/loop/notice", 1000);
	
    // Wait for video streams to be up
    BlockWhileWaitingForVideo();  
	
    cornHelp.Init(newArrival->im);
    
    // Create images
	view_im = cvCreateImage( cvSize(2*newArrival->width, newArrival->height), 8, IPL_PXL_BYTES );
	
	cvNamedWindow("LoopDetect", CV_WINDOW_AUTOSIZE);
	
    // Main loop
    printf("Entering main loop\n");

    int keyframe_done = -1;
    while (ros::ok())
    {
        char c = cvWaitKey(5);
        if (c == 'Q' || c == 'q')
            break;
            
		// Run a RANSAC
		CheckForLoopAndPubQ();
		
        // Get Images
        ros::spinOnce();
        
        // *****************************************
        // Check if we have unprocessed keyframes
        // *****************************************
        if ( keyframes.size() <= keyframe_done + 1)
            continue;       
        keyframe_done = keyframes.size() - 1;
		
		// Detect loop closure
        detectLoopClosure();
		
		// Publish edges for Toro
		publishNewFrame();
    }
	
    return 0;
}

