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
 * <ic2020_vodom/src/Keyframe.h>
 * 
 * revision 1.0
 */
 
#ifndef H_KEYFRAME
#define H_KEYFRAME

#include "ros/ros.h"
#include <stdio.h>
#include "cv.h"
#include <highgui.h>
 
#include "ic2020_vodom/keyframe.h"

struct PointColor {
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t wastedspace;
};

typedef struct surfdesc_ {
    float desc[64];
} surfdesc;

typedef struct feature_ {
    float point2D[2]; // 2D coordinate found in original image
    float point3D[3]; // 3D coordinate
} feature;

class Keyframe
{
	public:
	    Keyframe();
		Keyframe(const ic2020_vodom::keyframe::ConstPtr& msg);
		virtual ~Keyframe();
		
		void Init();
		void PublishKeyframe(ros::Publisher* ros_pub);
		
	    // Keyframe Number
	    unsigned int keyframe_num;
	    
	    // Rotation and Translation Data
	    float rotation[9];
	    float translation[3]; 
	    
	    // Image data
	    unsigned int height;
	    unsigned int width;
		IplImage* im;

        // 3D Point Cloud Data
        unsigned int numberOf3DPoints;
        unsigned int point_step; //used for getting point in single row based off x and y coords
        PointColor* points;
        
        // First set of Shi Tomasi Corners, relates this keyframe to the last one
        unsigned int numCorn1;
        std::vector<feature> corn1;
        std::vector<char> status; // indicates good pairs
        
        // First set of Shi Tomasi Corners, corners found in this keyframe to relate to next
        unsigned int numCorn2;
        std::vector<feature> corn2;

        // SURF Feature Set
        //unsigned int numSURF;
        std::vector<feature> features;
        std::vector<surfdesc> descBuffer;
        std::vector<int> surfMatches; // int is the index of the surf feature in the last keyframe
        
        // IMU
        float imux;
        float imuy;
        float imuz;
        
        // ****
        // RENDERER ONLY VARIABLE
        // ****
        bool global;
        
	private:
	
};

#endif
