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
 * <ic2020_toro/src/ic2020_toro.cpp>
 * 
 * revision 1.0
 */
 
 #include "ros/ros.h"
#include "std_msgs/String.h"

#include <highgui.h>
#include <cv.h>

#include <string>
#include <vector>

// Get Keyframe Message
#include "ic2020_vodom/keyframe.h"
#include "Keyframe.h"

#include "ic2020_render/rendupdate.h"
#include "KeyframeUpdater.h"

#include "ic2020_toro/newedge.h"
#include "ic2020_toro/loopnotice.h"

#include <iostream>

#include <sys/time.h>
#include <fstream>
#include "treeoptimizer3.hh"

using namespace std;
using namespace AISNavigation;

vector<TreeNode> keyframe_nodes;

TreeOptimizer3 pg;
int  iterations=100;
bool dumpError=true;
bool adaptiveRestart=false;
int  verboseLevel=0;
bool ignorePreconditioner=false;

TreeOptimizer3::EdgeCompareMode compareMode=EVComparator<TreeOptimizer3::Edge*>::CompareLevel;

float imu_rot[9];

double globTransfData[16] = {0.0}; // keep global info?
bool timeForALoopClose = false;

// obs_keyframe should be -1 if there is no previous obs
void AddEdge(int prime_keyframe, int obs_keyframe, float* rotation, float* translation, bool is_init)
{
    // Get Relative Data Matrices
    double relTransfData[16];
    double relTransfData2[16];
    CvMat relTransfMat = cvMat(4,4,CV_64FC1, &relTransfData[0]); //64F means double precision
    CvMat relTransfMat2 = cvMat(4,4,CV_64FC1, &relTransfData2[0]); //64F means double precision
    
    relTransfData[0]  = rotation[0];     relTransfData[1]  = rotation[1];     relTransfData[2]  = rotation[2];     relTransfData[3]  = translation[0];
    relTransfData[4]  = rotation[3];     relTransfData[5]  = rotation[4];     relTransfData[6]  = rotation[5];     relTransfData[7]  = translation[1];
    relTransfData[8]  = rotation[6];     relTransfData[9]  = rotation[7];     relTransfData[10] = rotation[8];     relTransfData[11] = translation[2];
    relTransfData[12] = 0.0f;            relTransfData[13] = 0.0f;            relTransfData[14] = 0.0f;            relTransfData[15] = 1.0f;
    if (!is_init)
    {
        imu_rot[0]  = 0.0f;     imu_rot[1]  = -1.0f;    imu_rot[2]  = 0.0f;
        imu_rot[3]  = 1.0f;     imu_rot[4]  = 0.0f;     imu_rot[5]  = 0.0f;
        imu_rot[6]  = 0.0f;     imu_rot[7]  = 0.0f;     imu_rot[8] = 1.0f;
    }
    cvInvert(&relTransfMat, &relTransfMat2, CV_LU);
    
    bool new_node = true;
    if (is_init)
    {
        int offset = keyframe_nodes[0].id;
        int index = prime_keyframe - offset;
        
        if (index < keyframe_nodes.size()) // add an edge to existing
            new_node = false;
    }
    
    // Add New Node!
    if (new_node == true)
    {
        CvMat globTransfMat = cvMat(4,4,CV_64FC1, &globTransfData[0]); //64F means double precision
        cvGEMM(&globTransfMat, &relTransfMat2, 1.0, NULL, 0.0, &globTransfMat, NULL);

        TreeNode temp;
        temp.id = prime_keyframe;
        
        temp.p.x = globTransfData[3];
        temp.p.y = globTransfData[7]; 
        temp.p.z = globTransfData[11];
        double pitch = atan2(-globTransfData[8], sqrt(pow(globTransfData[0],2.0)+pow(globTransfData[4],2.0)));
        double yaw = atan2(globTransfData[4]/cos(pitch), globTransfData[0]/cos(pitch));
        double roll = atan2(globTransfData[9]/cos(pitch), globTransfData[10]/cos(pitch));       
        temp.p.roll = roll; 
        temp.p.pitch = pitch; 
        temp.p.yaw = yaw;

        if (obs_keyframe >= 0)
        {
            TreeEdge tempedge;
            
            // Transposed Rotation
            double rel_pitch = atan2(-rotation[2], sqrt(pow(rotation[0],2.0)+pow(rotation[1],2.0)));
            double rel_yaw =   atan2(rotation[1]/cos(rel_pitch), rotation[0]/cos(rel_pitch));
            double rel_roll =  atan2(rotation[5]/cos(rel_pitch), rotation[8]/cos(rel_pitch));
            
            tempedge.id = obs_keyframe; 
            tempedge.p.x = relTransfData2[3];
            tempedge.p.y = relTransfData2[7]; 
            tempedge.p.z = relTransfData2[11]; 
            tempedge.p.roll= rel_roll; 
            tempedge.p.pitch= rel_pitch; 
            tempedge.p.yaw= rel_yaw;
            temp.edges.push_back(tempedge);
        }

        keyframe_nodes.push_back(temp);

    } 
    else if (obs_keyframe >= 0) // Add New Edge! 
    {
        int offset = keyframe_nodes[0].id;
        int index = prime_keyframe - offset;
        
        TreeEdge tempedge;
        
        // Transposed Rotation
        double rel_pitch = atan2(-rotation[2], sqrt(pow(rotation[0],2.0)+pow(rotation[1],2.0)));
        double rel_yaw =   atan2(rotation[1]/cos(rel_pitch), rotation[0]/cos(rel_pitch));
        double rel_roll =  atan2(rotation[5]/cos(rel_pitch), rotation[8]/cos(rel_pitch));
        
        tempedge.id = obs_keyframe; 
        tempedge.p.x = relTransfData2[3];
        tempedge.p.y = relTransfData2[7]; 
        tempedge.p.z = relTransfData2[11]; 
        tempedge.p.roll= rel_roll; 
        tempedge.p.pitch= rel_pitch; 
        tempedge.p.yaw= rel_yaw;
        
        if (keyframe_nodes[index].id == prime_keyframe)
            keyframe_nodes[index].edges.push_back(tempedge);
        else
            printf("WTF!! ADDING EDGE TO PRIME KEYFRAME, BUT KEYNUM != INDEX'D FRAME!!!");
    }
    
}

void loopCallback(const ic2020_toro::newedge::ConstPtr& msg)
{
printf("Loop Callback 1\n");
        
    // Get Data From Message
    int prime_keyframe = msg->prime_keyframe;
    int obs_keyframe = msg->obs_keyframe;

    // Get Rot and Trans Data
    float rotation[9];
    float translation[3];    

    int num_bytes;
    num_bytes = 9*sizeof(float);
    const float* temprot = reinterpret_cast<const float*>(&msg->rot[0]);
    memcpy(&rotation[0], temprot, num_bytes);

    num_bytes = 3*sizeof(float);
    const float* temptrans = reinterpret_cast<const float*>(&msg->trans[0]);
    memcpy(&translation[0], temptrans, num_bytes);
    
printf("Loop Callback 2\n");
    // Check For First Node Ever
    if (keyframe_nodes.size() == 0) {
printf("Loop Callback 2-1\n");
        AddEdge(prime_keyframe, obs_keyframe, &rotation[0], &translation[0], false);
    } else {
printf("Loop Callback 2-2\n");
        AddEdge(prime_keyframe, obs_keyframe, &rotation[0], &translation[0], true);
    }
printf("Loop Callback 3\n");        
}

void loopNoticeCallback(const ic2020_toro::loopnotice::ConstPtr& msg)
{
    printf("Got a Loop Notice\n");
    char notice = msg->close;
    timeForALoopClose = true;
}

void UpdateKeyframeNodes(std::vector<TreeNode>* updates)
{
    int offset = keyframe_nodes[0].id; // index = id - offset
    for (int k = 0; k < updates->size(); k++) {
        int i = (*updates)[k].id - offset;
        keyframe_nodes[i].p = (*updates)[k].p;
    }
}

void FillInRotTransData(TreeVertex* p, float* rot, float* trans)
{
    trans[0] = p->x;
    trans[1] = p->y;
    trans[2] = p->z;
    
    float cy = cos(p->yaw);
    float sy = sin(p->yaw);
    float cp = cos(p->pitch);
    float sp = sin(p->pitch);
    float cr = cos(p->roll);
    float sr = sin(p->roll);
    
    rot[0] = cy*cp;     rot[1] = cy*sp*sr - sy*cr;      rot[2] = cy*sp*cr + sy*sr;
    rot[3] = sy*cp;     rot[4] = sy*sp*sr + cy*cr;      rot[5] = sy*sp*cr - cy*sr;
    rot[6] = -sp;       rot[7] = cp*sr;                 rot[8] = cp*cr;
    
    /*CvMat rotMat = cvMat(3,3,CV_32FC1, &rot[0]);
    CvMat imuMat = cvMat(3,3,CV_32FC1, &imu_rot[0]);
    cvGEMM(&imuMat, &rotMat, 1.0, NULL, 0.0, &rotMat, NULL);*/
}

void PublishUpdates(ros::Publisher* ros_pub)
{
    KeyframeUpdater updater;
    for (int i = 0; i < keyframe_nodes.size(); i++) {
        update temp;
        temp.keyframe_num = keyframe_nodes[i].id;
        FillInRotTransData(&keyframe_nodes[i].p, &temp.rot[0], &temp.trans[0]);
        
        updater.updates.push_back(temp);
        
        cerr << "VECTOR3 = " << temp.keyframe_num << endl;
        cerr << "x = " << temp.trans[0] << endl;
        cerr << "y = " << temp.trans[1] << endl;
        cerr << "z = " << temp.trans[2] << endl;
        cerr << "yaw = " << keyframe_nodes[i].p.yaw << endl;
        cerr << "pitch = " << keyframe_nodes[i].p.pitch << endl;
        cerr << "roll = " << keyframe_nodes[i].p.roll << endl;
        cerr << endl;
    }
    updater.PublishUpdate(ros_pub);
}

int main (int argc, char** argv){
    struct timeval ts, te;

    ros::init(argc, argv, "ic2020_toro");
    ros::NodeHandle n;
    
//    ros::Subscriber vodom_sub = n.subscribe("/vodom/keyframes", 5, keyframeCallback);
    ros::Subscriber loop_sub = n.subscribe("/loop/edges", 1000, loopCallback);
    ros::Subscriber loopnote_sub = n.subscribe("/loop/notice", 1000, loopNoticeCallback);
    ros::Publisher keyupdate_pub = n.advertise<ic2020_render::rendupdate>("/toro/keyupdates", 5);   

    // Set Verbose
    pg.verboseLevel=verboseLevel;

    // Loop!
	while (ros::ok()) 
	{
	    // Get User Input
		char c = cvWaitKey(5);
		if (c == 'Q' || c == 'q')
			break;
		
		ros::spinOnce();
		
		// If thing is ten then lets do an optimization
		if (keyframe_nodes.size() > 2 && timeForALoopClose == true)
		{   
		    timeForALoopClose = false;
		    
		    printf("STARTING A LOOP CLOSURE\n");
		    
		    // Get Start of Init Time
            gettimeofday(&ts,0);

            // Load Nodes
            if ( !pg.loadNodes(&keyframe_nodes) ) {
                cerr << "FATAL ERROR: Could not load nodes!. Abrting." << endl;
                return 0;
            }
            cerr << "SYSTEM IS LOADED WITH #nodes: " << pg.vertices.size() << "  #edges: " << pg.edges.size() << endl; 
            
            // Set Random Stuff
            pg.restartOnDivergence=adaptiveRestart;
            
            // Build Tree
            cerr << "Incremental tree construction... ";
            pg.buildSimpleTree();

            // Initialize Guess
            cerr << "Computing initial guess from observations... ";
            pg.initializeOnTree();

            // Init Optimizer
            cerr << "Initializing the optimizer... ";
            pg.initializeTreeParameters();
            pg.initializeOptimization(compareMode);
            double l=pg.totalPathLength();
            int nEdges=pg.edges.size();
            double apl=l/(double)(nEdges);
            cerr << " Average path length=" << apl << endl;
            cerr << " Complexity of an iteration=" << l  << endl;

            // Get End of Init Time
            gettimeofday(&te,0);
            double loadtime=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
            cerr << "LOAD TIME= " << loadtime << " s." << endl;

            // Run Iterations!  
            gettimeofday(&ts,0);
            bool corrupted=false;
            cerr << "**** Starting optimization ****" << endl;
            for (int i=0; i < iterations; i++)
            {
                pg.iterate(0,ignorePreconditioner);

                if (dumpError){
                    // compute the error and dump it
                    double mte, mre, are, ate;
                    double error = pg.error(&mre, &mte, &are, &ate);
                    cerr << "iteration " << i << "  global error = " << error << endl;

                    if (mre>(M_PI/2)*(M_PI/2))
                        corrupted=true;
                    else
                        corrupted=false;
                }
            }
            gettimeofday(&te,0);
            double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
            cerr << "**** Optimization Done **** TOTAL TIME= " << dts << " s." << endl;

            // Update Keyframe Nodes from TORO Tree
            std::vector<TreeNode> updatedNodes;
            pg.saveNodes(&updatedNodes);
            UpdateKeyframeNodes(&updatedNodes);
            
            // Publish Out
            PublishUpdates(&keyupdate_pub);
		}
	}

    
    
    return 0;
}

