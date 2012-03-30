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
 * <ic2020_vodom/src/Keyframe.cpp>
 * 
 * revision 1.0
 */

#include "Keyframe.h"

Keyframe::Keyframe()
{
    Init();
}

Keyframe::Keyframe(const ic2020_vodom::keyframe::ConstPtr& msg)
{
    //printf("copying keyframe...\n");
    Init();
    int num_bytes;
    
    // Keyframe Number
    //printf("copying keyframe number %i...\n", msg->keyframe_num);
    keyframe_num = msg->keyframe_num;
    
    // Rotation and Translation Data
    //printf("Copying Rot and Trans\n");
    num_bytes = 9*sizeof(float);
    const float* temprot = reinterpret_cast<const float*>(&msg->rotation[0]);
    memcpy(&rotation[0], temprot, num_bytes);

    num_bytes = 3*sizeof(float);
    const float* temptrans = reinterpret_cast<const float*>(&msg->translation[0]);
    memcpy(&translation[0], temptrans, num_bytes);
    
    // Image data
    //printf("Copying Image\n");
    height = msg->height;
    width = msg->width;
    num_bytes = height*width*3*sizeof(char);
    const unsigned char* tempim = reinterpret_cast<const unsigned char*>(&msg->im[0]);
    im = cvCreateImage( cvSize(width, height), 8, 3);
    memcpy(&im->imageData[0], tempim, num_bytes);

    // 3D Point Cloud Data
    //printf("Copying 3D Points\n");
    numberOf3DPoints = msg->numberOf3DPoints;
    if (numberOf3DPoints > 0)
    {
        point_step = msg->point_step;
        points = new PointColor[numberOf3DPoints];
        const unsigned char* temp3d = reinterpret_cast<const unsigned char*>(&msg->points[0]);
        memcpy(&points[0], &temp3d[0], numberOf3DPoints*sizeof(PointColor));              
    }
    
    // First set of Shi Tomasi Corners, relates this keyframe to the last one
    //printf("Copying ShiTom1 Features\n");
    numCorn1 = msg->numCorn1;
    if (numCorn1 > 0) {
        corn1.resize(numCorn1);
        num_bytes = numCorn1*sizeof(feature);
        const unsigned char* tempCorn1 = reinterpret_cast<const unsigned char*>(&msg->corn1[0]);
        memcpy(&corn1[0], tempCorn1, num_bytes);
        
        status.resize(numCorn1);
        num_bytes = numCorn1*sizeof(char);
        const char* tempstat = reinterpret_cast<const char*>(&msg->status[0]);
        memcpy(&status[0], tempstat, num_bytes);
    } else {
        numCorn1 = 0;
    }
    
    // First set of Shi Tomasi Corners, corners found in this keyframe to relate to next
    //printf("Copying ShiTom2 Features\n");
    numCorn2 = msg->numCorn2;
    if (numCorn2 > 0) {
        corn2.resize(numCorn2);
        num_bytes = numCorn2*sizeof(feature);
        const unsigned char* tempCorn2 = reinterpret_cast<const unsigned char*>(&msg->corn2[0]);
        memcpy(&corn2[0], tempCorn2, num_bytes);
    } else {
        numCorn2 = 0;
    }

    // SURF Feature Set
    //printf("Copying SURF Features\n");
    if (msg->numSURF > 0) {
        features.resize(msg->numSURF);
        num_bytes = msg->numSURF*sizeof(feature);
        const unsigned char* tempfeat = reinterpret_cast<const unsigned char*>(&msg->features[0]);
        memcpy(&features[0], tempfeat, num_bytes);
        
        descBuffer.resize(msg->numSURF);
        num_bytes = msg->numSURF*sizeof(surfdesc);
        const unsigned char* tempdesc = reinterpret_cast<const unsigned char*>(&msg->descBuffer[0]);
        memcpy(&descBuffer[0], tempdesc, num_bytes);
        
        surfMatches.resize(msg->numSURF);
        num_bytes = msg->numSURF*sizeof(int);
        const unsigned char* tempmat = reinterpret_cast<const unsigned char*>(&msg->surfMatches[0]);
        memcpy(&surfMatches[0], tempmat, num_bytes);
    }

    imux = msg->imux;
    imuy = msg->imuy;
    imuz = msg->imuz;
    
    //printf("\n\n");
}

Keyframe::~Keyframe()
{
    //printf("Deconstructing Keyframe\n");
	cvReleaseImage( &im );
	if (points != 0) { delete [] points; }
}

void Keyframe::Init()
{
    keyframe_num = 0;
    
    // Rotation and Translation Data
    for (unsigned int i = 0; i < 9; i++) { rotation[i] = 0.0f; }
    for (unsigned int i = 0; i < 3; i++) { translation[i] = 0.0f; }
    
    // Image data
    height = 0;
    width = 0;
	im = 0;

    // 3D Point Cloud Data
    numberOf3DPoints = 0;
    point_step = 0; //used for getting point in single row based off x and y coords
    points = 0;
    
    // First set of Shi Tomasi Corners, relates this keyframe to the last one
    numCorn1 = 0;
    
    // First set of Shi Tomasi Corners, corners found in this keyframe to relate to next
    numCorn2 = 0;

    imux = 0.0f;
    imuy = 0.0f;
    imuz = 0.0f;

    // SURF Feature Set
    //numSURF = 0;
    //features = 0;
    
    // RENDERER ONLY
    global = false;
}

void Keyframe::PublishKeyframe(ros::Publisher* ros_pub)
{
    ic2020_vodom::keyframe msg;
    int num_bytes;

    // Keyframe Number
    msg.keyframe_num = keyframe_num;
    
    // Rotation and Translation Data    
    num_bytes = 9*sizeof(float);
    msg.rotation.resize(num_bytes);
    float* temprot = reinterpret_cast<float*>(&msg.rotation[0]);
    memcpy(temprot, &rotation[0], num_bytes);
    
    num_bytes = 3*sizeof(float);
    msg.translation.resize(num_bytes);
    float* temptrans = reinterpret_cast<float*>(&msg.translation[0]);
    memcpy(temptrans, &translation[0], num_bytes);
    
    // Image data
    msg.height = im->height;
    msg.width = im->width;
    num_bytes = msg.height*msg.width*3*sizeof(char);
    msg.im.resize(num_bytes);
    unsigned char* tempim = reinterpret_cast<unsigned char*>(&msg.im[0]);
    memcpy(tempim, &im->imageData[0], num_bytes);

    // 3D Point Cloud Data
    msg.numberOf3DPoints = numberOf3DPoints;
    msg.point_step = point_step;
    unsigned char *temp;
    num_bytes = msg.numberOf3DPoints*sizeof(PointColor);
    temp = new unsigned char[num_bytes];
    memcpy(temp, points, num_bytes);               
    std::vector<unsigned char> chdata(temp, temp + num_bytes/sizeof(unsigned char));        
    msg.points = chdata;
    delete [] temp;
    
    // First set of Shi Tomasi Corners, relates this keyframe to the last one
    if (numCorn1 > 0) {
        msg.numCorn1 = numCorn1;
        num_bytes = msg.numCorn1*sizeof(feature);
        msg.corn1.resize(num_bytes);
        unsigned char* tempCorn1 = reinterpret_cast<unsigned char*>(&msg.corn1[0]);
        memcpy(tempCorn1, &corn1[0], num_bytes);
        
        num_bytes = msg.numCorn1*sizeof(char);
        msg.status.resize(num_bytes);
        char* tempstat = reinterpret_cast<char*>(&msg.status[0]);
        memcpy(tempstat, &status[0], num_bytes);
    } else {
        msg.numCorn1 = 0;
    }
    
    // First set of Shi Tomasi Corners, corners found in this keyframe to relate to next
    if (numCorn2 > 0) {
        msg.numCorn2 = numCorn2;
        num_bytes = msg.numCorn2*sizeof(feature);
        msg.corn2.resize(num_bytes);
        unsigned char* tempCorn2 = reinterpret_cast<unsigned char*>(&msg.corn2[0]);
        memcpy(tempCorn2, &corn2[0], num_bytes);
    } else {
        msg.numCorn2 = 0;
    }

    // SURF Feature Set
    msg.numSURF = features.size();
    if (msg.numSURF > 0)
    {
        num_bytes = msg.numSURF*sizeof(feature);
        msg.features.resize(num_bytes);
        unsigned char* tempfeat = reinterpret_cast<unsigned char*>(&msg.features[0]);
        memcpy(tempfeat, &features[0], num_bytes);
        
        num_bytes = msg.numSURF*sizeof(surfdesc);
        msg.descBuffer.resize(num_bytes);
        unsigned char* tempdesc = reinterpret_cast<unsigned char*>(&msg.descBuffer[0]);
        memcpy(tempdesc, &descBuffer[0], num_bytes);
        
        num_bytes = msg.numSURF*sizeof(int);
        msg.surfMatches.resize(num_bytes);
        unsigned char* tempmat = reinterpret_cast<unsigned char*>(&msg.surfMatches[0]);
        memcpy(tempmat, &surfMatches[0], num_bytes);
    }
    
    msg.imux = imux;
    msg.imuy = imuy;
    msg.imuz = imuz;
    
    ros_pub->publish(msg);
}
