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
 * <ic2020_render/src/renderer.cpp>
 * 
 * revision 1.0
 */
 
 #include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <vector>

// Get Keyframe Message
#include "ic2020_vodom/keyframe.h"
#include "Keyframe.h"
#include "KeyframeUpdater.h"

#include "geometry_msgs/Pose2D.h"

// OpenGL
#include "GL/freeglut.h"
#include "GLCamera.h"
#include "Vector3.h"

std::vector<Keyframe*> keyframes;
KeyframeUpdater* keyUpdate = NULL;

unsigned int kin_pointstep;
float kin_points[640*480*4];

GLCamera glcam;
Vector3 ivec = Vector3(1.0f,0.0f,0.0f);
Vector3 jvec = Vector3(0.0f,0.0f,1.0f);

int axis_num = 0;
PointColor* axis_points = 0;

Vector3 initial_imu;
bool firstvodom = true;

void keyframeUpdateCallback(const ic2020_render::rendupdate::ConstPtr& msg)
{
    if (keyUpdate != NULL) { delete keyUpdate; }
    keyUpdate = new KeyframeUpdater(msg);
    
    if (keyframes.size() > 0) {
        // TODO CLEANUP INDEX ASSUMPTION
        unsigned int offset = (keyframes[0])->keyframe_num;
        for (unsigned int i = 0; i < keyUpdate->updates.size(); i++)
        {
            unsigned int index = keyUpdate->updates[i].keyframe_num - offset;
            if (index >= 0 && index < keyframes.size()) {
                (keyframes[index])->global = true;
                for (unsigned int j = 0; j < 9; j++) { (keyframes[index])->rotation[j] = keyUpdate->updates[i].rot[j]; }
                for (unsigned int j = 0; j < 3; j++) { (keyframes[index])->translation[j] = keyUpdate->updates[i].trans[j]; }
            }
        }
        
    }
}

#define LIMIT_DIST 3.5
Keyframe* filterNewKeyframe(const ic2020_vodom::keyframe::ConstPtr& msg)
{
    Keyframe* temp = new Keyframe(msg);
        
    for (int i = 0; i < temp->numberOf3DPoints; i++)
    {
        if (temp->points[i].z > LIMIT_DIST)
        {
            temp->points[i].x = NAN;
            temp->points[i].y = NAN;
            temp->points[i].z = NAN;
            temp->points[i].r = NAN;
            temp->points[i].g = NAN;
            temp->points[i].b = NAN;
        }
    }
    
    return temp;
}

void keyframeVodomCallback(const ic2020_vodom::keyframe::ConstPtr& msg)
{
    Keyframe* temp = filterNewKeyframe(msg);
    
    if (firstvodom)
    {
        firstvodom = false;
        Vector3 imu; imu.x = temp->imux; imu.y = temp->imuy; imu.z = temp->imuz; 
        imu.Normalize();
        float gamma = asin(imu.z);
        float alpha = atan2(imu.x,imu.y);
        initial_imu.z = alpha;
        initial_imu.x = gamma;
    }

    if (keyframes.size() > 0) {
        // TODO CLEANUP INDEX ASSUMPTION
        unsigned int offset = (keyframes[0])->keyframe_num;        
        unsigned int index = msg->keyframe_num - offset;
        if (index >= 0 && index < keyframes.size()) {
            Keyframe* temp2 = keyframes[index];
            keyframes[index] = temp;
            delete temp2;
            return;
        } else {
            keyframes.push_back(temp);
        }
    } else {    
        
        keyframes.push_back(temp);
    }
}

void keyframeCallback(const ic2020_vodom::keyframe::ConstPtr& msg)
{
    if (keyframes.size() > 0) {
        // TODO CLEANUP INDEX ASSUMPTION
        unsigned int offset = (keyframes[0])->keyframe_num;        
        unsigned int index = msg->keyframe_num - offset;
        if (index >= 0 && index < keyframes.size()) {
            Keyframe* temp = filterNewKeyframe(msg);
            Keyframe* temp2 = keyframes[index];
            keyframes[index] = temp;
            delete temp2;
            return;
        } else {
            Keyframe* temp = filterNewKeyframe(msg);
            keyframes.push_back(temp);
        }
    } else {    
        Keyframe* temp = filterNewKeyframe(msg);
        
        /*Vector3 imu; imu.x = temp->imux; imu.y = temp->imuy; imu.z = temp->imuz; 
        imu.Normalize();
        float gamma = asin(imu.z);
        float alpha = atan2(imu.x,imu.y);
        initial_imu.z = alpha;
        initial_imu.x = gamma;*/
        
        keyframes.push_back(temp);
    }
}

/*void loopCallback(const ic2020_toro::newedge::ConstPtr& msg)
{
printf("Loop Callback 1\n");
        
    // Get Data From Message
    int prime_keyframe = msg->prime_keyframe;
    int obs_keyframe = msg->obs_keyframe;
}*/

// INITIALIZATION
// **********************
void init()
{
    float point_every = 0.25f; //x meter spacing
    float length = 25.0f; //x meters long
    axis_num = 3*length/point_every;
    axis_points = new PointColor[axis_num];
    for (int i = 0; i < axis_num/3; i++)
    {
        axis_points[i*3].x = (float)i*point_every;      axis_points[i*3].y = 0.0f;                          axis_points[i*3].z = 0.0f; 
        axis_points[i*3+1].x = 0.0f;                    axis_points[i*3+1].y = (float)i*point_every;        axis_points[i*3+1].z = 0.0f; 
        axis_points[i*3+2].x = 0.0f;                    axis_points[i*3+2].y = 0.0f;                        axis_points[i*3+2].z = (float)i*point_every;
        
        if ( (float)(i*point_every) - (float)((int)(i*point_every)) > 1e-6)
        {
            axis_points[i*3].r = 255.0f;                    axis_points[i*3].g = 0.0f;                          axis_points[i*3].b = 0.0f;
            axis_points[i*3+1].r = 0.0f;                    axis_points[i*3+1].g = 255.0f;                      axis_points[i*3+1].b = 0.0f;
            axis_points[i*3+2].r = 0.0f;                    axis_points[i*3+2].g = 0.0f;                        axis_points[i*3+2].b = 255.0f;
        } else {
            axis_points[i*3].r = 255.0f;                    axis_points[i*3].g = 255.0f;                        axis_points[i*3].b = 255.0f;
            axis_points[i*3+1].r = 255.0f;                  axis_points[i*3+1].g = 255.0f;                      axis_points[i*3+1].b = 255.0f;
            axis_points[i*3+2].r = 255.0f;                  axis_points[i*3+2].g = 255.0f;                      axis_points[i*3+2].b = 255.0f;
        }              
    }

	glcam.Init();
	glcam.SetPosition(Vector3(0.0f,0.0f,-2.0f));
	glcam.SetViewDir(Vector3(0.0f,0.0f,1.0f));
	glcam.SetUpDir(Vector3(0.0f,-1.0f,0.0f));

	return;
}

// INITIALIZE OPEN GL
// **************************
void initGL(void)
{
	// Background color
	glClearColor(0.4, 0.47, 0.55, 0.0);

	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);

	// Configure Lighting
	GLfloat LightAmbient[] = {0.9f, 0.9f, 0.9f, 1.0f};
	GLfloat LightDiffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat LightSpecular[] = {0.5f, 0.5f, 0.5f, 1.0f};

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 90.0);

	//glEnable(GL_LIGHT0);
	//glEnable(GL_LIGHTING);

	// Enable various rending attributes
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_COLOR_MATERIAL);

	// Add specularity to all objects
	float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specReflection);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 30.0);
	
	// Configure Appearance of POINTS
	glPointSize(2.0);

    /*glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);*/
}

// RESIZE WINDOW
// ********************
void resize(int width, int height)
{
	glcam.resize(width, height);
}

int last_x = 0;
int last_y = 0;
int old_left_state = GLUT_UP;
void processMouse(int button, int state, int x, int y) {

	if (button == GLUT_LEFT_BUTTON && old_left_state == GLUT_UP && state == GLUT_DOWN) {
		old_left_state = GLUT_DOWN;
		last_x = x;
		last_y = y;
	}
	else if (button == GLUT_LEFT_BUTTON && old_left_state == GLUT_DOWN && state == GLUT_UP) {
		old_left_state = GLUT_UP;
	}
}

void processMouseActiveMotion(int x, int y) {
	if (old_left_state == GLUT_DOWN)
	{
		glcam.SetViewDir(glcam.GetViewDir()*1000 + glcam.GetRightDir()*(float)(x-last_x));
		last_x = x;
	}
	if (old_left_state == GLUT_DOWN)
	{
		glcam.SetViewDir(glcam.GetViewDir()*1000 - glcam.GetUpDir()*(float)(y-last_y));
		last_y = y;
	}
}

// KEYBOARD HANDLER
// ************************************************
void keyboard(unsigned char key, int x, int y)
{
	Vector3 temp;
	float speed = 0.4f;
	switch (key)
	{
        case 27:
            exit(1);
			break;


		case 'a':
			glcam.SetPosition(glcam.GetPosition() - glcam.GetRightDir()*speed);
			break;
		case 'd':
			glcam.SetPosition(glcam.GetPosition() + glcam.GetRightDir()*speed);
			break;
		case 'w':
			temp = (ivec*glcam.GetViewDir().Dot(ivec) + jvec*glcam.GetViewDir().Dot(jvec));
			temp.Normalize();
			glcam.SetPosition(glcam.GetPosition() + temp*speed);
			break;
		case 's':
			temp = (ivec*glcam.GetViewDir().Dot(ivec) + jvec*glcam.GetViewDir().Dot(jvec));
			temp.Normalize();
			glcam.SetPosition(glcam.GetPosition() - temp*speed);
			break;
		case 'r':
			glcam.SetPosition(glcam.GetPosition() + glcam.GetUpDir()*speed);
			break;
		case 'f':
			glcam.SetPosition(glcam.GetPosition() - glcam.GetUpDir()*speed);
			break;
    }
}

// DRAW FUNCTION
// ***********************
void display(void)
{
    
// Clear Buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Load Modelview
	glMatrixMode(GL_MODELVIEW);

	// Set camera direction and position
	glLoadIdentity();

	glcam.LookAt();

	// Lighting stuff
	//GLfloat LightPosition[] = {0.0f,0.0f,10.0f,1.0f};
	//GLfloat LightDirection[] = {1.0f, 1.0f, -1.0f};
	//glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	//glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, LightDirection);
	
	// DRAW AXIS POINTS
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    if (axis_num > 0)
    {
        glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(PointColor), ((uint8_t*)axis_points+12));   
        glVertexPointer(3, GL_FLOAT, sizeof(PointColor), ((float*)axis_points));
        glDrawArrays(GL_POINTS, 0, axis_num);
    }
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);	 
    
	// Fix with IMU
    GLfloat imumat[16];
    double alpha = initial_imu.z;
    double gamma = initial_imu.x;
    imumat[0] = cos(alpha);    imumat[4] = -sin(alpha)*cos(gamma);    imumat[8]  = sin(alpha)*sin(gamma);     imumat[12] = 0.0f;
    imumat[1] = sin(alpha);    imumat[5] = cos(alpha)*cos(gamma);     imumat[9]  = -cos(alpha)*sin(gamma);    imumat[13] = 0.0f;
    imumat[2] = 0.0f;          imumat[6] = sin(gamma);                imumat[10] = cos(gamma);                imumat[14] = 0.0f;
    imumat[3] = 0.0f;          imumat[7] = 0.0f;                      imumat[11] = 0.0f;                      imumat[15] = 1.0f;
    glMultMatrixf(imumat);
	
	glPushMatrix();
	
	// DRAW KEYFRAMES
	for (unsigned int i = 0; i < keyframes.size(); i++)
	{ 
	    // Change Coordinate Frame
	    if (keyframes[i]->global == true)
	    {
	       glPopMatrix();
	       glPushMatrix();
	    }
	    
	    GLfloat mat[16];
	    mat[0] = keyframes[i]->rotation[0];     mat[4] = keyframes[i]->rotation[1];     mat[8]  = keyframes[i]->rotation[2];     mat[12] = keyframes[i]->translation[0];
	    mat[1] = keyframes[i]->rotation[3];     mat[5] = keyframes[i]->rotation[4];     mat[9]  = keyframes[i]->rotation[5];     mat[13] = keyframes[i]->translation[1];
	    mat[2] = keyframes[i]->rotation[6];     mat[6] = keyframes[i]->rotation[7];     mat[10] = keyframes[i]->rotation[8];     mat[14] = keyframes[i]->translation[2];
	    mat[3] = 0.0f;                          mat[7] = 0.0f;                          mat[11] = 0.0f;                          mat[15] = 1.0f;
	    glMultMatrixf(mat);
	    
	    // Draw Camera Volume
	    if (i == keyframes.size() - 1)
	    {
	        float w = keyframes[i]->im->width;
	        float h = keyframes[i]->im->height;
	        float fx = 519.0f;
	        float fy = 519.0f;
	        float cx = 335.0f;
	        float cy = 267.0f;
	        float z = 1.0f;
	        float x1 = z*(0.0f-cx)/fx;
	        float y1 = z*(0.0f-cy)/fy;
	        float x2 = z*(w-cx)/fx;
	        float y2 = z*(0.0f-cy)/fy;
	        float x3 = z*(w-cx)/fx;
	        float y3 = z*(h-cy)/fy;
	        float x4 = z*(0.0f-cx)/fx;
	        float y4 = z*(h-cy)/fy;
	        
	        glBegin(GL_LINES);
	            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(x1, y1, z);
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(x2, y2, z);
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(x3, y3, z);
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(x4, y4, z);

                glVertex3f(x1, y1, z);
                glVertex3f(x2, y2, z);
                glVertex3f(x2, y2, z);
                glVertex3f(x3, y3, z);
                glVertex3f(x3, y3, z);
                glVertex3f(x4, y4, z);
                glVertex3f(x4, y4, z);
                glVertex3f(x1, y1, z);
            glEnd();
	    }
	    
	    // Draw 3-Axis Thing
	    glBegin(GL_LINES);
	        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.1f, 0.0f, 0.0f);
            glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.1f, 0.0f);
            glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.1f);
        glEnd();
	    
	    // Draw Points
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        if (keyframes[i]->numberOf3DPoints > 0)
        {
            glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(PointColor), ((uint8_t*)keyframes[i]->points+12));   
            glVertexPointer(3, GL_FLOAT, sizeof(PointColor), ((float*)keyframes[i]->points));
            glDrawArrays(GL_POINTS, 0, keyframes[i]->numberOf3DPoints);
        }
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);   
        
        // Draw Matched Features  
        /*for (int j = 0; j < keyframes[i]->features.size(); j++)
        {
            glPushMatrix();
                if (i%3 == 0)
                    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
                else if (i%3 == 1)
                    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
                else
                    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
                    
                glTranslatef(keyframes[i]->features[j].point3D[0], keyframes[i]->features[j].point3D[1], keyframes[i]->features[j].point3D[2]); // x,y,z
                glBegin(GL_POINTS);
                    glutWireSphere(0.01, 6, 6);
                glEnd( );
            glPopMatrix();
        }*/
    }
    glPopMatrix();   
 
	// Swap buffers
	glutSwapBuffers();
}

// CALCULATIONS
// ***************************************
void mainLoop(int v)
{

    ros::spinOnce();

	glutTimerFunc(10, mainLoop, 0);

	return;
}

int main( int argc, char** argv )
{

    ros::init(argc, argv, "ic2020_renderer");
    ros::NodeHandle n;
    
    ros::Subscriber render_sub = n.subscribe("/flow/contframes", 5, keyframeCallback);
    ros::Subscriber render_sub2 = n.subscribe("/vodom/keyframes", 5, keyframeVodomCallback);
    ros::Subscriber rendupdate_sub = n.subscribe("/toro/keyupdates", 5, keyframeUpdateCallback);
    //ros::Subscriber edges_sub = n.subscribe("/loop/edges", 1000, loopCallback);
    
	int screenwidth = 800;
	int screenheight = 600;

	// Call for variable initialization
	init();

	//INIT OPENGL
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(screenwidth, screenheight);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("VSLAM");

	initGL();

	// Set display handler, keyboard handler, reshape handler
	glutDisplayFunc(display); // not sure if this does anything.. rendering seems to occur in idle
	glutIdleFunc(display);
	glutReshapeFunc(resize);

	glutMouseFunc(processMouse);
	glutMotionFunc(processMouseActiveMotion);
	glutKeyboardFunc(keyboard);

	// ** The use of this timer in collaboration with the timer
	//    found at the end of the mainLoop function acts as a while(1) infinite loop.
	glutTimerFunc(1, mainLoop, 0);

	// Begins looping the content set up in glut
	glutMainLoop();

    // CLEAN UP
    if (axis_points != 0) { delete [] axis_points; }
    axis_points = 0;

	return 0;
}

