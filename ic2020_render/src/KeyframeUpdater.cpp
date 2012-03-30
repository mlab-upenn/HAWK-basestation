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
 * <ic2020_render/src/KeyframeUpdater.cpp>
 * 
 * revision 1.0
 */
 
 #include "KeyframeUpdater.h"

KeyframeUpdater::KeyframeUpdater() {}
KeyframeUpdater::~KeyframeUpdater() {}

KeyframeUpdater::KeyframeUpdater(const ic2020_render::rendupdate::ConstPtr& msg)
{
    int num_bytes;
    
    // Update Data
    updates.resize(msg->numOfUpdates);
    num_bytes = msg->numOfUpdates*sizeof(update);
    const unsigned char* temp = reinterpret_cast<const unsigned char*>(&msg->data[0]);
    memcpy(&updates[0], temp, num_bytes);
}

void KeyframeUpdater::PublishUpdate(ros::Publisher* ros_pub)
{
    ic2020_render::rendupdate msg;
    int num_bytes;

    msg.numOfUpdates = updates.size();
    if (msg.numOfUpdates > 0)
    {
        num_bytes = msg.numOfUpdates*sizeof(update);
        msg.data.resize(num_bytes);
        unsigned char* temp = reinterpret_cast<unsigned char*>(&msg.data[0]);
        memcpy(temp, &updates[0], num_bytes);
        
        ros_pub->publish(msg);
    }
}
