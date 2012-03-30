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
 * <ic2020_render/src/GLCamera.h>
 * 
 * revision 1.0
 */
 
 #ifndef H_GLCAMERA
#define H_GLCAMERA

#include "Vector3.h"

class GLCamera {

public:
	GLCamera();
	~GLCamera();

	void Init();

	void LookAt();

	void SetPosition(Vector3 pos);
	void SetViewDir(Vector3 vdir);
	void SetUpDir(Vector3 udir);

	void resize(int width, int height);
	
	Vector3 GetPosition();
	Vector3 GetViewDir();
	Vector3 GetUpDir();
	Vector3 GetRightDir();

private:
	double nearZ;                                        //near clipping plane
	double farZ;                                        //far clipping plane

	Vector3 _position;
	Vector3 _viewDir;
	Vector3 _upDir;
};

#endif
