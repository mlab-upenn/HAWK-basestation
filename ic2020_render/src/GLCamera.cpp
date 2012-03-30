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
 * <ic2020_render/src/GLCamera.cpp>
 * 
 * revision 1.0
 */
 
#include "GLCamera.h"
#include "GL/glut.h"

#include <math.h>
#include "Constants.h"

GLCamera::GLCamera() {}

GLCamera::~GLCamera() {}

void GLCamera::Init()
{
	nearZ = 0.01;                                        //near clipping plane
	farZ = 300.0;                                        //far clipping plane
}

void GLCamera::LookAt()
{
	gluLookAt( 
		(GLdouble) _position.x,
		(GLdouble) _position.y,
		(GLdouble) _position.z,
		(GLdouble) _position.x+_viewDir.x,
		(GLdouble) _position.y+_viewDir.y,
		(GLdouble) _position.z+_viewDir.z,
		(GLdouble) _upDir.x,
		(GLdouble) _upDir.y,
		(GLdouble) _upDir.z
	);
}

void GLCamera::resize(int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (float)width/(float)height, nearZ, farZ);
}

Vector3 GLCamera::GetPosition()
{
	return _position;
}

Vector3 GLCamera::GetViewDir()
{
	return _viewDir;
}

Vector3 GLCamera::GetUpDir()
{
	return _upDir;
}

Vector3 GLCamera::GetRightDir()
{
	Vector3 right = _viewDir.Cross(_upDir);
	right.Normalize();
	return right;
}

void GLCamera::SetPosition(Vector3 pos)
{
	_position = pos;
}

void GLCamera::SetViewDir(Vector3 vdir)
{
	_viewDir = vdir;
	_viewDir.Normalize();
}

void GLCamera::SetUpDir(Vector3 udir)
{
	_upDir = udir;
	_upDir.Normalize();
}
