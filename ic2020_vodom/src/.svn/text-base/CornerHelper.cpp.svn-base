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
 * <ic2020_vodom/src/CornerHelper.cpp>
 * 
 * revision 1.0
 */
 
 #include "CornerHelper.h"

CornerHelper::CornerHelper()
{
    cornersA = NULL;
    cornersB = NULL;
}

CornerHelper::~CornerHelper(){}

void CornerHelper::Init(IplImage* exampleIm)
{
    imgA = cvCreateImage( cvSize(exampleIm->width, exampleIm->height), 8, 1 );
    imgB = cvCreateImage( cvSize(exampleIm->width, exampleIm->height), 8, 1 );

    img_sz = cvGetSize( exampleIm );
    pyr_sz = cvSize( exampleIm->width+8, exampleIm->height/3 );
    eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
    tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
    pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
    pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
    cornersA = new CvPoint2D32f[ MAX_CORNERS ];
    cornersB = new CvPoint2D32f[ MAX_CORNERS ];
    win_size = 10;
    corner_count = MAX_CORNERS;
}

void CornerHelper::FindFeatures(Keyframe* kA, Keyframe* kB, std::vector<feature>* cornA, std::vector<feature>* cornB, std::vector<char>* pairs)
{
    cornA->clear();
    cornB->clear();
    pairs->clear();     

    cvCvtColor( kA->im, imgA, CV_BGR2GRAY );
    cvCvtColor( kB->im, imgB, CV_BGR2GRAY );	     
    
    if (kA->numCorn2 > 0)
    {
        corner_count = kA->numCorn2;
        for( int i = 0; i < corner_count; i++ ) {
            cornersA[i].x = kA->corn2[i].point2D[0];
            cornersA[i].y = kA->corn2[i].point2D[1];
        }
    } else {
        corner_count = MAX_CORNERS;
        cvGoodFeaturesToTrack(imgA, eig_image, tmp_image, cornersA,
            &corner_count, 0.01, 5.0, 0, 3, 0, 0.04 );
    }
    
double tt3 = (double)cvGetTickCount();	

    cvCalcOpticalFlowPyrLK(imgA, imgB, pyrA, pyrB, 
        cornersA, cornersB, corner_count,
        cvSize( win_size, win_size ), 5, features_found, feature_errors,
        cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ), 0
    );
    
tt3 = (double)cvGetTickCount() - tt3;
printf( "TIME: CalcOpticalFlow Time = %gms\n", tt3/(cvGetTickFrequency()*1000.));
    
    for( int i = 0; i < corner_count; i++ ) {
        if( features_found[i] == 0 )
            continue;
        if( feature_errors[i] > 550 ) {
            features_found[i] = 0;
            continue;
        }
        if ( cvRound(cornersA[i].y) < 0 || cvRound(cornersA[i].y) > kA->im->height ||
             cvRound(cornersA[i].x) < 0 || cvRound(cornersA[i].x) > kA->im->width  ||
             cvRound(cornersB[i].y) < 0 || cvRound(cornersB[i].y) > kB->im->height ||
             cvRound(cornersB[i].x) < 0 || cvRound(cornersB[i].x) > kB->im->width  )
        {
            features_found[i] = 0;
            continue;
        }
        
        feature fa;
        fa.point2D[0] = cornersA[i].x;
        fa.point2D[1] = cornersA[i].y;
        
        feature fb;
        fb.point2D[0] = cornersB[i].x;
        fb.point2D[1] = cornersB[i].y;
        
        PointColor pA = kA->points[cvRound(fa.point2D[1])*kA->point_step + 
                                   cvRound(fa.point2D[0])];
        PointColor pB = kB->points[cvRound(fb.point2D[1])*kB->point_step + 
                                   cvRound(fb.point2D[0])];

        if ( !(pA.z > 0.25f) || !(pB.z > 0.25f) || 
           (fabs(pA.z) > 50.0f) || (fabs(pB.z) > 50.0f) ||
           (fabs(pA.y) > 50.0f) || (fabs(pB.y) > 50.0f) ||
           (fabs(pA.x) > 50.0f) || (fabs(pB.x) > 50.0f) )
        {
            features_found[i] = 0;
            continue;
        }
        
        fa.point3D[0] = pA.x;
        fa.point3D[1] = pA.y;
        fa.point3D[2] = pA.z;
                     
        fb.point3D[0] = pB.x;
        fb.point3D[1] = pB.y;
        fb.point3D[2] = pB.z;
        
        cornA->push_back(fa);
        cornB->push_back(fb);
        pairs->push_back(features_found[i]);
    }

}

