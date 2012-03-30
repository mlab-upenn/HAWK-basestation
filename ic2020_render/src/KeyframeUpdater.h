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
 * <ic2020_render/src/KeyframeUpdater.h>
 * 
 * revision 1.0
 */
 
 #ifndef H_KEYFRAMEUPDATER
#define H_KEYFRAMEUPDATER

#include "ros/ros.h"
#include <stdio.h>

#include "ic2020_render/rendupdate.h"

typedef struct update_ {
    unsigned int keyframe_num;
    float rot[9];
    float trans[3];
} update;

class KeyframeUpdater
{
	public:
	    KeyframeUpdater();
		KeyframeUpdater(const ic2020_render::rendupdate::ConstPtr& msg);
		virtual ~KeyframeUpdater();
		
		void PublishUpdate(ros::Publisher* ros_pub);
		
        std::vector<update> updates;
        
	private:
	
};

#endif
