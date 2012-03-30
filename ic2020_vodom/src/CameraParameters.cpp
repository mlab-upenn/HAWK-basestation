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
 * <ic2020_vodom/src/CameraParameters.cpp>
 * 
 * revision 1.0
 */

#include "CameraParameters.h"

CameraParameters::CameraParameters(CvSize _size, int left_or_right, unsigned int channels)
{
	row_alligned_image = cvCreateImage( _size, 8, channels);

	CvFileStorage*  fs;
	if (left_or_right == LEFT)
		fs = cvOpenFileStorage("intrinsics_left.xml",0,CV_STORAGE_READ);
	else
		fs = cvOpenFileStorage("intrinsics_right.xml",0,CV_STORAGE_READ);
	_M = (CvMat*)cvReadByName(fs,0,"M");
	_D = (CvMat*)cvReadByName(fs,0,"D");
	cvReleaseFileStorage(&fs);

	fs = cvOpenFileStorage("extrinsics.xml",0,CV_STORAGE_READ);
	if (left_or_right == LEFT)
	{
		_R = (CvMat*)cvReadByName(fs,0,"R1");
		_P = (CvMat*)cvReadByName(fs,0,"P1");
	}
	else
	{
		_R = (CvMat*)cvReadByName(fs,0,"R2");
		_P = (CvMat*)cvReadByName(fs,0,"P2");
	}
	cvReleaseFileStorage(&fs);

	mx = cvCreateMat( _size.height, _size.width, CV_32F );
	my = cvCreateMat( _size.height, _size.width, CV_32F );

	cvInitUndistortRectifyMap(_M, _D, _R, _P, mx, my);
}

CameraParameters::~CameraParameters()
{
	cvReleaseImage( &row_alligned_image );
}

CvMat* CameraParameters::GetCameraMatrix()
{
    return _M;
}

CvMat* CameraParameters::GetDistortionCoeff()
{
    return _D;
}

IplImage* CameraParameters::RowAllignImage(IplImage* raw)
{
	cvRemap(raw, row_alligned_image, mx, my);
	return row_alligned_image;
}

void CameraParameters::UndistortPoints(CvMat* src, CvMat* dst)
{
    cvUndistortPoints(src, dst, _M, _D, _R, _P);
}
