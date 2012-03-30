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
 * <ic2020_vodom/src/VisualOdometry.cpp>
 * 
 * revision 1.0
 */
 
 #include "VisualOdometry.h"

#include <cstdlib>
#include "Vector3.h"

void VisualOdometry::RotateAndTrans (double* orig_point, double* rot, double* trans, double* new_point) {

    CvMat p1 = cvMat(3,1,CV_64FC1, &orig_point[0]); //64F means double
    CvMat R = cvMat(3,3,CV_64FC1, &rot[0]); //64F means double
    CvMat p2 = cvMat(3,1,CV_64FC1, &new_point[0]);
   
    cvGEMM(&R, &p1, 1.0, NULL, 0.0, &p2, NULL);
    new_point[0] += trans[0];
    new_point[1] += trans[1];
    new_point[2] += trans[2];
}

void VisualOdometry::RotateAndTransf (float* orig_point, float* rot, float* trans, float* new_point) {

    CvMat p1 = cvMat(3,1,CV_32FC1, &orig_point[0]); //64F means double
    CvMat R = cvMat(3,3,CV_32FC1, &rot[0]); //64F means double
    CvMat p2 = cvMat(3,1,CV_32FC1, &new_point[0]);
   
    cvGEMM(&R, &p1, 1.0, NULL, 0.0, &p2, NULL);
    new_point[0] += trans[0];
    new_point[1] += trans[1];
    new_point[2] += trans[2];
}

bool VisualOdometry::RANSAC6DReproj (std::vector<feature>* features_a, std::vector<feature>* features_b, 
                                     std::vector<unsigned int>* pairs, std::vector<unsigned int>* filtered_pairs)
{
// Initialize
    srand ( time(NULL) );
    int i1, i2, i3, num;
    i1 = i2 = i3 = 0;
    num = pairs->size()/2;

    std::vector<unsigned int> curr_pairs;

    // less than 10 points can't possibly produce anything decent
    if (num > 9) 
    {
        int iter_count = 0;        
        while (iter_count < RANSAC_MAX_ITER)
        {
            iter_count++;
            
            curr_pairs.clear();
        
            i1 = i2 = i3 = rand() % num;
            while (i2 == i1) { i2 = rand() % num; }
            while (i3 == i1 || i3 == i2) { i3 = rand() % num; }
            
            unsigned int ia1 = (*pairs)[2*i1+0];
            unsigned int ib1 = (*pairs)[2*i1+1];
            unsigned int ia2 = (*pairs)[2*i2+0];
            unsigned int ib2 = (*pairs)[2*i2+1];
            unsigned int ia3 = (*pairs)[2*i3+0];
            unsigned int ib3 = (*pairs)[2*i3+1];
            
            Vector3 a1, a2, a3, b1, b2, b3;
            a1.x = (*features_a)[ia1].point3D[0]; a1.y = (*features_a)[ia1].point3D[1]; a1.z = (*features_a)[ia1].point3D[2];
            a2.x = (*features_a)[ia2].point3D[0]; a2.y = (*features_a)[ia2].point3D[1]; a2.z = (*features_a)[ia2].point3D[2];
            a3.x = (*features_a)[ia3].point3D[0]; a3.y = (*features_a)[ia3].point3D[1]; a3.z = (*features_a)[ia3].point3D[2];
            b1.x = (*features_b)[ib1].point3D[0]; b1.y = (*features_b)[ib1].point3D[1]; b1.z = (*features_b)[ib1].point3D[2];
            b2.x = (*features_b)[ib2].point3D[0]; b2.y = (*features_b)[ib2].point3D[1]; b2.z = (*features_b)[ib2].point3D[2];
            b3.x = (*features_b)[ib3].point3D[0]; b3.y = (*features_b)[ib3].point3D[1]; b3.z = (*features_b)[ib3].point3D[2];
            
            Vector3 ax, ay, az, bx, by, bz;
            ax = a2 - a1; ax.Normalize();
            ay = (a3 - a1) - ax*(ax.Dot(a3-a1)); ay.Normalize();
            az = ax.Cross(ay);
            bx = b2 - b1; bx.Normalize();
            by = (b3 - b1) - bx*(bx.Dot(b3-b1)); by.Normalize();
            bz = bx.Cross(by);
            
            double rotA[9];
            double rotB[9];
            double rot[9];
            CvMat RA = cvMat(3,3,CV_64FC1, &rotA[0]); //64F means double
            CvMat RB = cvMat(3,3,CV_64FC1, &rotB[0]); //64F means double
            CvMat R = cvMat(3,3,CV_64FC1, &rot[0]); //64F means double
            
            rotA[0] = ax.x; rotA[1] = ay.x; rotA[2] = az.x;
            rotA[3] = ax.y; rotA[4] = ay.y; rotA[5] = az.y;
            rotA[6] = ax.z; rotA[7] = ay.z; rotA[8] = az.z;
            
            rotB[0] = bx.x; rotB[1] = by.x; rotB[2] = bz.x;
            rotB[3] = bx.y; rotB[4] = by.y; rotB[5] = bz.y;
            rotB[6] = bx.z; rotB[7] = by.z; rotB[8] = bz.z;
            
            cvGEMM(&RB, &RA, 1.0, NULL, 0.0, &R, CV_GEMM_B_T); // b = R*a
            
            // Find Translation
            Vector3 trans;
            double a1dat[3] = {0};
            double a1rot[3] = {0};
            CvMat A1 = cvMat(3,1,CV_64FC1, &a1dat[0]); //64F means double
            CvMat A1Rot = cvMat(3,1,CV_64FC1, &a1rot[0]); //64F means double
            a1dat[0] = a1.x; a1dat[1] = a1.y; a1dat[2] = a1.z;
                
            cvGEMM(&R, &A1, 1.0, NULL, 0.0, &A1Rot, NULL);
            trans.x = b1.x - a1rot[0];
            trans.y = b1.y - a1rot[1];
            trans.z = b1.z - a1rot[2];
            
            // SURF Pairs
            for (unsigned int i = 0; i < pairs->size()/2; i++)
            {
                unsigned int a = (*pairs)[2*i+0];
                unsigned int b = (*pairs)[2*i+1];
                
                double aa[3] = {0};
                double ab[3] = {0};
                CvMat AA = cvMat(3,1,CV_64FC1, &aa[0]); //64F means double
                CvMat AB = cvMat(3,1,CV_64FC1, &ab[0]); //64F means double
                
                aa[0] = (*features_a)[a].point3D[0];
                aa[1] = (*features_a)[a].point3D[1];
                aa[2] = (*features_a)[a].point3D[2];
                
                cvGEMM(&R, &AA, 1.0, NULL, 0.0, &AB, NULL);

                ab[0] += trans.x;
                ab[1] += trans.y;
                ab[2] += trans.z;

                Vector3 ap, bp;
                ap.x = ab[0]; ap.y = ab[1]; ap.z = ab[2];
                bp.x = (*features_b)[b].point3D[0]; 
                bp.y = (*features_b)[b].point3D[1]; 
                bp.z = (*features_b)[b].point3D[2];
                
                double bb[3] = {0};
                bb[0] = (*features_b)[b].point3D[0];
                bb[1] = (*features_b)[b].point3D[1];
                bb[2] = (*features_b)[b].point3D[2];
                
                double fx = 519.00864543;
                double fy = 519.00864543;
                double cx = 335.16694839;
                double cy = 267.36505989;
                double ua = int((ab[0]/ab[2])*fx + cx + 0.5);
                double va = int((ab[1]/ab[2])*fy + cy + 0.5);
                double ub = int((bb[0]/bb[2])*fx + cx + 0.5);
                double vb = int((bb[1]/bb[2])*fy + cy + 0.5);
 
                printf("ub  vb  %f %f\n", (*features_b)[b].point2D[0], (*features_b)[b].point2D[1]);
                printf("ub2 vb2 %f %f\n", ub, vb);
                printf("ua  va  %f %f\n", ua, va);
                
                //if ( (ap-bp).Length() < RANSAC_3D_THRESH )
                if ( fabs(ua-ub) + fabs(va-vb) < RANSAC_REPROJ_THRESH && (ap-bp).Length() < RANSAC_THRESH_ERROR_PER_M*bp.z)
                {
                    curr_pairs.push_back(a);
                    curr_pairs.push_back(b);
                }
            }
            
            if ( (curr_pairs.size()) > (filtered_pairs->size()) )
            {
                (*filtered_pairs) = curr_pairs;
            }
        }
    } else {
        return false;
    }
    return true;  
}

bool VisualOdometry::GetRotFromRnd3(std::vector<feature>* features_a, std::vector<feature>* features_b, double* rot, double* trans)
{

    int i1, i2, i3, num;
    i1 = i2 = i3 = 0;
    num = features_a->size();
    
    if (num > 9) 
    {
        i1 = i2 = i3 = rand() % num;
        while (i2 == i1) { i2 = rand() % num; }
        while (i3 == i1 || i3 == i2) { i3 = rand() % num; }
        
        Vector3 a1, a2, a3, b1, b2, b3;
        a1.x = (*features_a)[i1].point3D[0]; a1.y = (*features_a)[i1].point3D[1]; a1.z = (*features_a)[i1].point3D[2];
        a2.x = (*features_a)[i2].point3D[0]; a2.y = (*features_a)[i2].point3D[1]; a2.z = (*features_a)[i2].point3D[2];
        a3.x = (*features_a)[i3].point3D[0]; a3.y = (*features_a)[i3].point3D[1]; a3.z = (*features_a)[i3].point3D[2];
        b1.x = (*features_b)[i1].point3D[0]; b1.y = (*features_b)[i1].point3D[1]; b1.z = (*features_b)[i1].point3D[2];
        b2.x = (*features_b)[i2].point3D[0]; b2.y = (*features_b)[i2].point3D[1]; b2.z = (*features_b)[i2].point3D[2];
        b3.x = (*features_b)[i3].point3D[0]; b3.y = (*features_b)[i3].point3D[1]; b3.z = (*features_b)[i3].point3D[2];
        
        Vector3 ax, ay, az, bx, by, bz;
        ax = a2 - a1; ax.Normalize();
        ay = (a3 - a1) - ax*(ax.Dot(a3-a1)); ay.Normalize();
        az = ax.Cross(ay);
        bx = b2 - b1; bx.Normalize();
        by = (b3 - b1) - bx*(bx.Dot(b3-b1)); by.Normalize();
        bz = bx.Cross(by);
        
        double rotA[9];
        double rotB[9];
        CvMat RA = cvMat(3,3,CV_64FC1, &rotA[0]); //64F means double
        CvMat RB = cvMat(3,3,CV_64FC1, &rotB[0]); //64F means double
        CvMat R = cvMat(3,3,CV_64FC1, &rot[0]); //64F means double
        
        rotA[0] = ax.x; rotA[1] = ay.x; rotA[2] = az.x;
        rotA[3] = ax.y; rotA[4] = ay.y; rotA[5] = az.y;
        rotA[6] = ax.z; rotA[7] = ay.z; rotA[8] = az.z;
        
        rotB[0] = bx.x; rotB[1] = by.x; rotB[2] = bz.x;
        rotB[3] = bx.y; rotB[4] = by.y; rotB[5] = bz.y;
        rotB[6] = bx.z; rotB[7] = by.z; rotB[8] = bz.z;
        
        cvGEMM(&RB, &RA, 1.0, NULL, 0.0, &R, CV_GEMM_B_T); // b = R*a
        
        // Find Translation
        double a1dat[3] = {0};
        double a1rot[3] = {0};
        CvMat A1 = cvMat(3,1,CV_64FC1, &a1dat[0]); //64F means double
        CvMat A1Rot = cvMat(3,1,CV_64FC1, &a1rot[0]); //64F means double
        a1dat[0] = a1.x; a1dat[1] = a1.y; a1dat[2] = a1.z;
            
        cvGEMM(&R, &A1, 1.0, NULL, 0.0, &A1Rot, NULL);
        trans[0] = b1.x - a1rot[0];
        trans[1] = b1.y - a1rot[1];
        trans[2] = b1.z - a1rot[2];
    } else {
        return false;
    }
    return true;
}

bool VisualOdometry::RANSAC6D      (std::vector<feature>* features_a, std::vector<feature>* features_b, 
                                    std::vector<unsigned int>* pairs, std::vector<unsigned int>* filtered_pairs,
                                    feature* corn_a, feature* corn_b, char* corn_pairs, unsigned int num_corn_pairs,
                                    std::vector<unsigned int>* filtered_corn_pairs)
{
    filtered_pairs->clear();
    filtered_corn_pairs->clear();
    
    // Initialize
    srand ( time(NULL) );
    int i1, i2, i3, num;
    i1 = i2 = i3 = 0;
    num = pairs->size()/2;

    std::vector<unsigned int> curr_pairs;
    std::vector<unsigned int> curr_corn_pairs;
    unsigned int best_num_good_corn = 0;

    // less than 10 points can't possibly produce anything decent
    if (num > 9) 
    {
        int iter_count = 0;        
        while (iter_count < RANSAC_MAX_ITER)
        {
            iter_count++;
            
            curr_pairs.clear();
            curr_corn_pairs.clear();
        
            i1 = i2 = i3 = rand() % num;
            while (i2 == i1) { i2 = rand() % num; }
            while (i3 == i1 || i3 == i2) { i3 = rand() % num; }
            
            unsigned int ia1 = (*pairs)[2*i1+0];
            unsigned int ib1 = (*pairs)[2*i1+1];
            unsigned int ia2 = (*pairs)[2*i2+0];
            unsigned int ib2 = (*pairs)[2*i2+1];
            unsigned int ia3 = (*pairs)[2*i3+0];
            unsigned int ib3 = (*pairs)[2*i3+1];
            
            Vector3 a1, a2, a3, b1, b2, b3;
            a1.x = (*features_a)[ia1].point3D[0]; a1.y = (*features_a)[ia1].point3D[1]; a1.z = (*features_a)[ia1].point3D[2];
            a2.x = (*features_a)[ia2].point3D[0]; a2.y = (*features_a)[ia2].point3D[1]; a2.z = (*features_a)[ia2].point3D[2];
            a3.x = (*features_a)[ia3].point3D[0]; a3.y = (*features_a)[ia3].point3D[1]; a3.z = (*features_a)[ia3].point3D[2];
            b1.x = (*features_b)[ib1].point3D[0]; b1.y = (*features_b)[ib1].point3D[1]; b1.z = (*features_b)[ib1].point3D[2];
            b2.x = (*features_b)[ib2].point3D[0]; b2.y = (*features_b)[ib2].point3D[1]; b2.z = (*features_b)[ib2].point3D[2];
            b3.x = (*features_b)[ib3].point3D[0]; b3.y = (*features_b)[ib3].point3D[1]; b3.z = (*features_b)[ib3].point3D[2];
            
            Vector3 ax, ay, az, bx, by, bz;
            ax = a2 - a1; ax.Normalize();
            ay = (a3 - a1) - ax*(ax.Dot(a3-a1)); ay.Normalize();
            az = ax.Cross(ay);
            bx = b2 - b1; bx.Normalize();
            by = (b3 - b1) - bx*(bx.Dot(b3-b1)); by.Normalize();
            bz = bx.Cross(by);
            
            double rotA[9];
            double rotB[9];
            double rot[9];
            CvMat RA = cvMat(3,3,CV_64FC1, &rotA[0]); //64F means double
            CvMat RB = cvMat(3,3,CV_64FC1, &rotB[0]); //64F means double
            CvMat R = cvMat(3,3,CV_64FC1, &rot[0]); //64F means double
            
            rotA[0] = ax.x; rotA[1] = ay.x; rotA[2] = az.x;
            rotA[3] = ax.y; rotA[4] = ay.y; rotA[5] = az.y;
            rotA[6] = ax.z; rotA[7] = ay.z; rotA[8] = az.z;
            
            rotB[0] = bx.x; rotB[1] = by.x; rotB[2] = bz.x;
            rotB[3] = bx.y; rotB[4] = by.y; rotB[5] = bz.y;
            rotB[6] = bx.z; rotB[7] = by.z; rotB[8] = bz.z;
            
            cvGEMM(&RB, &RA, 1.0, NULL, 0.0, &R, CV_GEMM_B_T); // b = R*a
            
            // Find Translation
            Vector3 trans;
            double a1dat[3] = {0};
            double a1rot[3] = {0};
            CvMat A1 = cvMat(3,1,CV_64FC1, &a1dat[0]); //64F means double
            CvMat A1Rot = cvMat(3,1,CV_64FC1, &a1rot[0]); //64F means double
            a1dat[0] = a1.x; a1dat[1] = a1.y; a1dat[2] = a1.z;
                
            cvGEMM(&R, &A1, 1.0, NULL, 0.0, &A1Rot, NULL);
            trans.x = b1.x - a1rot[0];
            trans.y = b1.y - a1rot[1];
            trans.z = b1.z - a1rot[2];
            
            // SURF Pairs
            for (unsigned int i = 0; i < pairs->size()/2; i++)
            {
                unsigned int a = (*pairs)[2*i+0];
                unsigned int b = (*pairs)[2*i+1];
                
                double p1[3] = {0};
                double p2[3] = {0};
                CvMat P1 = cvMat(3,1,CV_64FC1, &p1[0]); //64F means double
                CvMat P2 = cvMat(3,1,CV_64FC1, &p2[0]); //64F means double
                
                p1[0] = (*features_a)[a].point3D[0];
                p1[1] = (*features_a)[a].point3D[1];
                p1[2] = (*features_a)[a].point3D[2];
                
                cvGEMM(&R, &P1, 1.0, NULL, 0.0, &P2, NULL);
                
                Vector3 ap, bp;
                ap.x = p2[0]; ap.y = p2[1]; ap.z = p2[2];
                ap = ap + trans;
                bp.x = (*features_b)[b].point3D[0]; 
                bp.y = (*features_b)[b].point3D[1]; 
                bp.z = (*features_b)[b].point3D[2];
                
                //if ( (ap-bp).Length() < RANSAC_3D_THRESH )
                if ( (ap-bp).Length() < RANSAC_THRESH_ERROR_PER_M*bp.z )
                {
                    curr_pairs.push_back(a);
                    curr_pairs.push_back(b);
                }
            }
            
            // ST Pairs
            unsigned int num_good_corn = 0;
            for (unsigned int i = 0; i < num_corn_pairs; i++)
            {   
                if (corn_pairs[i] > 0)
                {
                    double p1[3] = {0};
                    double p2[3] = {0};
                    CvMat P1 = cvMat(3,1,CV_64FC1, &p1[0]); //64F means double
                    CvMat P2 = cvMat(3,1,CV_64FC1, &p2[0]); //64F means double
                    
                    p1[0] = (corn_a)[i].point3D[0];
                    p1[1] = (corn_a)[i].point3D[1];
                    p1[2] = (corn_a)[i].point3D[2];
                    
                    cvGEMM(&R, &P1, 1.0, NULL, 0.0, &P2, NULL);
                    
                    Vector3 ap, bp;
                    ap.x = p2[0]; ap.y = p2[1]; ap.z = p2[2];
                    ap = ap + trans;
                    
                    bp.x = (corn_b)[i].point3D[0]; 
                    bp.y = (corn_b)[i].point3D[1]; 
                    bp.z = (corn_b)[i].point3D[2];
                    
                    //if ( (ap-bp).Length() < RANSAC_3D_THRESH )
                    if ( (ap-bp).Length() < RANSAC_THRESH_ERROR_PER_M*bp.z ) {
                        num_good_corn++;
                        curr_corn_pairs.push_back(1);
                    } else {
                        curr_corn_pairs.push_back(0);
                    }
                } else {
                    curr_corn_pairs.push_back(0);
                }
            }
            
            if ( (curr_pairs.size()+num_good_corn) > (filtered_pairs->size()+best_num_good_corn) )
            {
                (*filtered_pairs) = curr_pairs;
                (*filtered_corn_pairs) = curr_corn_pairs;
                best_num_good_corn = num_good_corn;
            }
        }
    } else {
        return false;
    }

    return true;                
}

bool VisualOdometry::RANSAC6DFastCorn(feature* corn_a, feature* corn_b, char* corn_pairs, unsigned int num_corn_pairs,
                                      std::vector<unsigned int>* filtered_corn_pairs,
                                      float imwid, float imhei, unsigned int xbins, unsigned int ybins, unsigned int maxperbin)
{
    srand ( time(NULL) );
    
    filtered_corn_pairs->clear();

    // Binning
    std::vector<feature> matchesA;
    std::vector<feature> matchesB;
    
    unsigned int *bin_count = new unsigned int[xbins*ybins];
    for (unsigned int i = 0; i < xbins*ybins; i++) { bin_count[i] = 0; }
    for (unsigned int i = 0; i < num_corn_pairs; i++) {
        if (corn_pairs[i] > 0)
        {        
            float lx = corn_b[i].point2D[0];
            float ly = corn_b[i].point2D[1];
            if (lx < 0.0f || lx > imwid || ly < 0.0f || ly > imhei) continue;
            unsigned int xbin = (unsigned int)(lx/(imwid/(float)xbins));
            unsigned int ybin = (unsigned int)(ly/(imhei/(float)ybins));
            if (bin_count[ybin*ybins+xbin] < maxperbin)
            {
                bin_count[ybin*ybins+xbin]++;
                matchesA.push_back(corn_a[i]);
                matchesB.push_back(corn_b[i]);
            }
        }
    }
    delete [] bin_count;

    // less than 10 points can't possibly produce anything decent
    if (num_corn_pairs > 9) 
    {
        int iter_count = 0;
        double bestrot[9];
        double besttrans[3];  
        int bestcount = 0;    
        while (iter_count < RANSAC_MAX_ITER)
        {
            int count = 0;
            iter_count++;
            
            double rot[9];
            double trans[3];
            
            GetRotFromRnd3(&matchesA, &matchesB, &rot[0], &trans[0]);
            CvMat R = cvMat(3,3,CV_64FC1, &rot[0]); //64F means double
            
            // Pairs
            for (unsigned int i = 0; i < matchesA.size(); i++)
            {   
                double p1[3] = {0};
                double p2[3] = {0};
                CvMat P1 = cvMat(3,1,CV_64FC1, &p1[0]); //64F means double
                CvMat P2 = cvMat(3,1,CV_64FC1, &p2[0]); //64F means double
                
                p1[0] = matchesA[i].point3D[0];
                p1[1] = matchesA[i].point3D[1];
                p1[2] = matchesA[i].point3D[2];
                
                cvGEMM(&R, &P1, 1.0, NULL, 0.0, &P2, NULL);
                
                Vector3 ap, bp;
                ap.x = p2[0]; ap.y = p2[1]; ap.z = p2[2];
                ap.x = ap.x + trans[0];
                ap.y = ap.y + trans[1];
                ap.z = ap.z + trans[2];
                bp.x = matchesB[i].point3D[0]; 
                bp.y = matchesB[i].point3D[1]; 
                bp.z = matchesB[i].point3D[2];
                
                //if ( (ap-bp).Length() < RANSAC_3D_THRESH )
                if ( (ap-bp).Length() < RANSAC_THRESH_ERROR_PER_M*bp.z )
                {
                    count++;
                }
            }
            
            if ( count > bestcount )
            {
                bestcount = count;
                for (unsigned int r = 0; r < 9; r++) { bestrot[r] = rot[r]; }
                for (unsigned int r = 0; r < 3; r++) { besttrans[r] = trans[r]; }
            }
        }
        
        CvMat R = cvMat(3,3,CV_64FC1, &bestrot[0]); //64F means double

        // ST Pairs
        unsigned int num_good_corn = 0;
        for (unsigned int i = 0; i < num_corn_pairs; i++)
        {   
            if (corn_pairs[i] > 0)
            {
                double p1[3] = {0};
                double p2[3] = {0};
                CvMat P1 = cvMat(3,1,CV_64FC1, &p1[0]); //64F means double
                CvMat P2 = cvMat(3,1,CV_64FC1, &p2[0]); //64F means double
                
                p1[0] = (corn_a)[i].point3D[0];
                p1[1] = (corn_a)[i].point3D[1];
                p1[2] = (corn_a)[i].point3D[2];
                
                cvGEMM(&R, &P1, 1.0, NULL, 0.0, &P2, NULL);
                
                Vector3 ap, bp;
                ap.x = p2[0]; ap.y = p2[1]; ap.z = p2[2];

                ap.x = ap.x + besttrans[0];
                ap.y = ap.y + besttrans[1];
                ap.z = ap.z + besttrans[2];
                
                bp.x = (corn_b)[i].point3D[0]; 
                bp.y = (corn_b)[i].point3D[1]; 
                bp.z = (corn_b)[i].point3D[2];
                
                //if ( (ap-bp).Length() < RANSAC_3D_THRESH )
                if ( (ap-bp).Length() < RANSAC_THRESH_ERROR_PER_M*bp.z ) {
                    filtered_corn_pairs->push_back(1);
                } else {
                    filtered_corn_pairs->push_back(0);
                }
            } else {
                filtered_corn_pairs->push_back(0);
            }
        }
        
    } else {
        return false;
    }

    return true;       
}

bool VisualOdometry::RANSAC6DFast  (std::vector<feature>* features_a, std::vector<feature>* features_b, 
                                    std::vector<unsigned int>* pairs, std::vector<unsigned int>* filtered_pairs,
                                    feature* corn_a, feature* corn_b, char* corn_pairs, unsigned int num_corn_pairs,
                                    std::vector<unsigned int>* filtered_corn_pairs,
                                    float imwid, float imhei, unsigned int xbins, unsigned int ybins, unsigned int maxperbin)
{
    srand ( time(NULL) );
    
    filtered_pairs->clear();
    filtered_corn_pairs->clear();
    
    // Initialize
    int num = pairs->size()/2;

    // Binning
    std::vector<feature> matchesA;
    std::vector<feature> matchesB;
    
    unsigned int *bin_count = new unsigned int[xbins*ybins];
    for (unsigned int i = 0; i < xbins*ybins; i++) { bin_count[i] = 0; }
    for (unsigned int i = 0; i < pairs->size()/2; i++) {
        unsigned int a = (*pairs)[2*i+0];
        unsigned int b = (*pairs)[2*i+1];
        
        float lx = (*features_b)[b].point2D[0];
        float ly = (*features_b)[b].point2D[1];
        if (lx < 0.0f || lx > imwid || ly < 0.0f || ly > imhei) continue;
        unsigned int xbin = (unsigned int)(lx/(imwid/(float)xbins));
        unsigned int ybin = (unsigned int)(ly/(imhei/(float)ybins));
        if (bin_count[ybin*ybins+xbin] < maxperbin)
        {
            //bin_count[ybin*ybins+xbin]++;
            matchesA.push_back((*features_a)[a]);
            matchesB.push_back((*features_b)[b]);
        }
    }
    for (unsigned int i = 0; i < num_corn_pairs; i++) {
        if (corn_pairs[i] > 0)
        {        
            float lx = corn_b[i].point2D[0];
            float ly = corn_b[i].point2D[1];
            if (lx < 0.0f || lx > imwid || ly < 0.0f || ly > imhei) continue;
            unsigned int xbin = (unsigned int)(lx/(imwid/(float)xbins));
            unsigned int ybin = (unsigned int)(ly/(imhei/(float)ybins));
            if (bin_count[ybin*ybins+xbin] < maxperbin)
            {
                bin_count[ybin*ybins+xbin]++;
                matchesA.push_back(corn_a[i]);
                matchesB.push_back(corn_b[i]);
            }
        }
    }
    delete [] bin_count;

    // less than 10 points can't possibly produce anything decent
    if (num > 9) 
    {
        int iter_count = 0;
        double bestrot[9];
        double besttrans[3];  
        int bestcount = 0;    
        while (iter_count < RANSAC_MAX_ITER)
        {
            int count = 0;
            iter_count++;
            
            double rot[9];
            double trans[3];
            
            GetRotFromRnd3(&matchesA, &matchesB, &rot[0], &trans[0]);
            CvMat R = cvMat(3,3,CV_64FC1, &rot[0]); //64F means double
            
            // Pairs
            for (unsigned int i = 0; i < matchesA.size(); i++)
            {   
                double p1[3] = {0};
                double p2[3] = {0};
                CvMat P1 = cvMat(3,1,CV_64FC1, &p1[0]); //64F means double
                CvMat P2 = cvMat(3,1,CV_64FC1, &p2[0]); //64F means double
                
                p1[0] = matchesA[i].point3D[0];
                p1[1] = matchesA[i].point3D[1];
                p1[2] = matchesA[i].point3D[2];
                
                cvGEMM(&R, &P1, 1.0, NULL, 0.0, &P2, NULL);
                
                Vector3 ap, bp;
                ap.x = p2[0]; ap.y = p2[1]; ap.z = p2[2];
                ap.x = ap.x + trans[0];
                ap.y = ap.y + trans[1];
                ap.z = ap.z + trans[2];
                bp.x = matchesB[i].point3D[0]; 
                bp.y = matchesB[i].point3D[1]; 
                bp.z = matchesB[i].point3D[2];
                
                //if ( (ap-bp).Length() < RANSAC_3D_THRESH )
                if ( (ap-bp).Length() < RANSAC_THRESH_ERROR_PER_M*bp.z )
                {
                    count++;
                }
            }
            
            if ( count > bestcount )
            {
                bestcount = count;
                for (unsigned int r = 0; r < 9; r++) { bestrot[r] = rot[r]; }
                for (unsigned int r = 0; r < 3; r++) { besttrans[r] = trans[r]; }
            }
        }
        
        CvMat R = cvMat(3,3,CV_64FC1, &bestrot[0]); //64F means double
        
        // SURF Pairs
        for (unsigned int i = 0; i < pairs->size()/2; i++)
        {
            unsigned int a = (*pairs)[2*i+0];
            unsigned int b = (*pairs)[2*i+1];
            
            double p1[3] = {0};
            double p2[3] = {0};
            CvMat P1 = cvMat(3,1,CV_64FC1, &p1[0]); //64F means double
            CvMat P2 = cvMat(3,1,CV_64FC1, &p2[0]); //64F means double
            
            p1[0] = (*features_a)[a].point3D[0];
            p1[1] = (*features_a)[a].point3D[1];
            p1[2] = (*features_a)[a].point3D[2];
            
            cvGEMM(&R, &P1, 1.0, NULL, 0.0, &P2, NULL);
            
            Vector3 ap, bp;
            ap.x = p2[0]; ap.y = p2[1]; ap.z = p2[2];
            ap.x = ap.x + besttrans[0];
            ap.y = ap.y + besttrans[1];
            ap.z = ap.z + besttrans[2];
            bp.x = (*features_b)[b].point3D[0]; 
            bp.y = (*features_b)[b].point3D[1]; 
            bp.z = (*features_b)[b].point3D[2];
            
            //if ( (ap-bp).Length() < RANSAC_3D_THRESH )
            if ( (ap-bp).Length() < RANSAC_THRESH_ERROR_PER_M*bp.z )
            {
                filtered_pairs->push_back(a);
                filtered_pairs->push_back(b);
            }
        }
        
        // ST Pairs
        unsigned int num_good_corn = 0;
        for (unsigned int i = 0; i < num_corn_pairs; i++)
        {   
            if (corn_pairs[i] > 0)
            {
                double p1[3] = {0};
                double p2[3] = {0};
                CvMat P1 = cvMat(3,1,CV_64FC1, &p1[0]); //64F means double
                CvMat P2 = cvMat(3,1,CV_64FC1, &p2[0]); //64F means double
                
                p1[0] = (corn_a)[i].point3D[0];
                p1[1] = (corn_a)[i].point3D[1];
                p1[2] = (corn_a)[i].point3D[2];
                
                cvGEMM(&R, &P1, 1.0, NULL, 0.0, &P2, NULL);
                
                Vector3 ap, bp;
                ap.x = p2[0]; ap.y = p2[1]; ap.z = p2[2];

                ap.x = ap.x + besttrans[0];
                ap.y = ap.y + besttrans[1];
                ap.z = ap.z + besttrans[2];
                
                bp.x = (corn_b)[i].point3D[0]; 
                bp.y = (corn_b)[i].point3D[1]; 
                bp.z = (corn_b)[i].point3D[2];
                
                //if ( (ap-bp).Length() < RANSAC_3D_THRESH )
                if ( (ap-bp).Length() < RANSAC_THRESH_ERROR_PER_M*bp.z ) {
                    filtered_corn_pairs->push_back(1);
                } else {
                    filtered_corn_pairs->push_back(0);
                }
            } else {
                filtered_corn_pairs->push_back(0);
            }
        }
        
    } else {
        return false;
    }

    return true;                
}

// Input, two feature sets, already paired. Rotation[9], Trans[3]
// Output, rot and trans will be filled
void VisualOdometry::ArunLeastSquares2D(std::vector<feature>* features_a, std::vector<feature>* features_b,
                             double* rotation_data, double* trans_data) {
    // Calculate Centroid A
    double a_centroid[2];
    double b_centroid[2];
    double a_size = features_a->size();
    double b_size = features_b->size();
        
    a_centroid[0] = 0.0f;
    a_centroid[1] = 0.0f;
    for (unsigned int i = 0; i < a_size; i++)
    {
        a_centroid[0] += (*features_a)[i].point3D[0];
        a_centroid[1] += (*features_a)[i].point3D[2];
    }
    a_centroid[0] /= a_size;
    a_centroid[1] /= a_size;

    // Calculate Centroid B
    b_centroid[0] = 0.0f;
    b_centroid[1] = 0.0f;
    for (unsigned int i = 0; i < b_size; i++)
    {
        b_centroid[0] += (*features_b)[i].point3D[0];
        b_centroid[1] += (*features_b)[i].point3D[2];
    }
    b_centroid[0] /= b_size;
    b_centroid[1] /= b_size;

    double htdata[4] = {0.0f};
    double hdata[4] = {0.0f};
    double q1data[2] = {0.0f};
    double q2data[2] = {0.0f};
    CvMat Htotal = cvMat(2,2,CV_64FC1, &htdata[0]); //64F means double
    CvMat H = cvMat(2,2,CV_64FC1, &hdata[0]); //64F means double
    CvMat q1 = cvMat(2,1,CV_64FC1, &q1data[0]);
    CvMat q2 = cvMat(2,1,CV_64FC1, &q2data[0]);	

    // Sum Up q's
    for (unsigned int i = 0; i < a_size; i++)
    {
        q1data[0] = (*features_b)[i].point3D[0] - b_centroid[0];
        q1data[1] = (*features_b)[i].point3D[2] - b_centroid[1];
        q2data[0] = (*features_a)[i].point3D[0] - a_centroid[0];
        q2data[1] = (*features_a)[i].point3D[2] - a_centroid[1];
        
        cvGEMM(&q1, &q2, 1.0, NULL, 0.0, &H, CV_GEMM_B_T);//NULL);               
        
        for (unsigned int s = 0; s < 4; s++) { htdata[s] = htdata[s] + hdata[s]; } // sum
    }

    double utdata[4] = {0};
    double adata[4] = {0};
    double vdata[4] = {0};
    CvMat Ut = cvMat(2,2,CV_64FC1, &utdata[0]); //64F means double
    CvMat A = cvMat(2,2,CV_64FC1, &adata[0]); //64F means double
    CvMat V = cvMat(2,2,CV_64FC1, &vdata[0]); //64F means double
    cvSVD(&Htotal, &A, &Ut, &V, CV_SVD_MODIFY_A | CV_SVD_U_T); // hope this works

    // Find Rotation
    double rdata[4] = {0};
    CvMat rotation = cvMat(2,2,CV_64FC1, &rdata[0]); //64F means double
    cvGEMM(&V, &Ut, 1.0, NULL, 0.0, &rotation, NULL);
    
    // Calculate determinate
    float rotdet =   rdata[0]*rdata[3]-rdata[1]*rdata[2];
                
    // If det(x) is -1.0, then there is a reflection due to coplanar points
    if (fabs(rotdet + 1.0f) < 1e-6) {
        printf("REFLECTION IN LEAST SQUARES! \n");
        vdata[0+1] = -vdata[0+1];
        vdata[2+1] = -vdata[2+1];
        cvGEMM(&V, &Ut, 1.0, NULL, 0.0, &rotation, NULL);
    }

    // Find Translation
    double translation[3] = {0};
    double datacentRot[2] = {0};
    CvMat centRot = cvMat(2,1,CV_64FC1, &datacentRot[0]); //64F means double
    CvMat newCentMat = cvMat(2,1,CV_64FC1, &b_centroid[0]); //64F means double
    cvGEMM(&rotation, &newCentMat, 1.0, NULL, 0.0, &centRot, NULL);
    translation[0] = a_centroid[0] - datacentRot[0];
    translation[1] = 0.0f;
    translation[2] = a_centroid[1] - datacentRot[1];
    
    printf("2Drot: %f %f \n", rdata[0], rdata[1]);
    printf("2Drot: %f %f \n", rdata[2], rdata[3]);
    
    rotation_data[0] = rdata[0];    rotation_data[1] = 0.0f;    rotation_data[2] = rdata[1];
    rotation_data[3] = 0.0f;        rotation_data[4] = 1.0f;    rotation_data[5] = 0.0f;
    rotation_data[6] = rdata[2];    rotation_data[7] = 0.0f;    rotation_data[8] = rdata[3];
    for (unsigned int i = 0; i < 3; i++) { trans_data[i] = translation[i]; }
}

// Input, two feature sets, already paired. Rotation[9], Trans[3]
// Output, rot and trans will be filled
void VisualOdometry::ArunLeastSquares(std::vector<feature>* features_a, std::vector<feature>* features_b,
                             double* rotation_data, double* trans_data) {
    // Calculate Centroid A
    double a_centroid[3];
    double b_centroid[3];
    double a_size = features_a->size();
    double b_size = features_b->size();
        
    a_centroid[0] = 0.0f;
    a_centroid[1] = 0.0f;
    a_centroid[2] = 0.0f;
    for (unsigned int i = 0; i < a_size; i++)
    {
        a_centroid[0] += (*features_a)[i].point3D[0];
        a_centroid[1] += (*features_a)[i].point3D[1];
        a_centroid[2] += (*features_a)[i].point3D[2];
    }
    a_centroid[0] /= a_size;
    a_centroid[1] /= a_size;
    a_centroid[2] /= a_size;

    // Calculate Centroid B
    b_centroid[0] = 0.0f;
    b_centroid[1] = 0.0f;
    b_centroid[2] = 0.0f;
    for (unsigned int i = 0; i < b_size; i++)
    {
        b_centroid[0] += (*features_b)[i].point3D[0];
        b_centroid[1] += (*features_b)[i].point3D[1];
        b_centroid[2] += (*features_b)[i].point3D[2];
    }
    b_centroid[0] /= b_size;
    b_centroid[1] /= b_size;
    b_centroid[2] /= b_size;

    double htdata[9] = {0.0f};
    double hdata[9] = {0.0f};
    double q1data[3] = {0.0f};
    double q2data[3] = {0.0f};
    CvMat Htotal = cvMat(3,3,CV_64FC1, &htdata[0]); //64F means double
    CvMat H = cvMat(3,3,CV_64FC1, &hdata[0]); //64F means double
    CvMat q1 = cvMat(3,1,CV_64FC1, &q1data[0]);
    CvMat q2 = cvMat(3,1,CV_64FC1, &q2data[0]);	

    // Sum Up q's
    for (unsigned int i = 0; i < a_size; i++)
    {
        q1data[0] = (*features_b)[i].point3D[0] - b_centroid[0];
        q1data[1] = (*features_b)[i].point3D[1] - b_centroid[1];
        q1data[2] = (*features_b)[i].point3D[2] - b_centroid[2];
        q2data[0] = (*features_a)[i].point3D[0] - a_centroid[0];
        q2data[1] = (*features_a)[i].point3D[1] - a_centroid[1];
        q2data[2] = (*features_a)[i].point3D[2] - a_centroid[2];
        
        cvGEMM(&q1, &q2, 1.0, NULL, 0.0, &H, CV_GEMM_B_T);//NULL);               
        
        for (unsigned int s = 0; s < 9; s++) { htdata[s] = htdata[s] + hdata[s]; } // sum
    }

    double utdata[9] = {0};
    double adata[9] = {0};
    double vdata[9] = {0};
    CvMat Ut = cvMat(3,3,CV_64FC1, &utdata[0]); //64F means double
    CvMat A = cvMat(3,3,CV_64FC1, &adata[0]); //64F means double
    CvMat V = cvMat(3,3,CV_64FC1, &vdata[0]); //64F means double
    cvSVD(&Htotal, &A, &Ut, &V, CV_SVD_MODIFY_A | CV_SVD_U_T); // hope this works

    // Find Rotation
    double rdata[9] = {0};
    CvMat rotation = cvMat(3,3,CV_64FC1, &rdata[0]); //64F means double
    cvGEMM(&V, &Ut, 1.0, NULL, 0.0, &rotation, NULL);
    
    // Calculate determinate
    float rotdet =   rdata[0]*rdata[3+1]*rdata[6+2]
                   + rdata[1]*rdata[3+2]*rdata[6+0]
                   + rdata[2]*rdata[3+0]*rdata[6+1]
                   - rdata[0]*rdata[3+2]*rdata[6+1]
                   - rdata[1]*rdata[3+0]*rdata[6+2]
                   - rdata[2]*rdata[3+1]*rdata[6+0];
                
    // If det(x) is -1.0, then there is a reflection due to coplanar points
    if (fabs(rotdet + 1.0f) < 1e-6) {
        printf("REFLECTION IN LEAST SQUARES! \n");
        vdata[0+2] = -vdata[0+2];
        vdata[3+2] = -vdata[3+2];
        vdata[6+2] = -vdata[6+2];
        cvGEMM(&V, &Ut, 1.0, NULL, 0.0, &rotation, NULL);
    }

    // Find Translation
    double translation[3] = {0};
    double datacentRot[3] = {0};
    CvMat centRot = cvMat(3,1,CV_64FC1, &datacentRot[0]); //64F means double
    CvMat newCentMat = cvMat(3,1,CV_64FC1, &b_centroid[0]); //64F means double
    cvGEMM(&rotation, &newCentMat, 1.0, NULL, 0.0, &centRot, NULL);
    translation[0] = a_centroid[0] - datacentRot[0];
    translation[1] = a_centroid[1] - datacentRot[1];
    translation[2] = a_centroid[2] - datacentRot[2];
    
    for (unsigned int i = 0; i < 9; i++) { rotation_data[i] = rdata[i]; }
    for (unsigned int i = 0; i < 3; i++) { trans_data[i] = translation[i]; }
}

void VisualOdometry::ArunLeastSquares(std::vector<Vector3>* features_a, std::vector<Vector3>* features_b,
                             double* rotation_data, double* trans_data) {
    // Calculate Centroid A
    double a_centroid[3];
    double b_centroid[3];
    double a_size = features_a->size();
    double b_size = features_b->size();
        
    a_centroid[0] = 0.0f;
    a_centroid[1] = 0.0f;
    a_centroid[2] = 0.0f;
    for (unsigned int i = 0; i < a_size; i++)
    {
        a_centroid[0] += (*features_a)[i].x;
        a_centroid[1] += (*features_a)[i].y;
        a_centroid[2] += (*features_a)[i].z;
    }
    a_centroid[0] /= a_size;
    a_centroid[1] /= a_size;
    a_centroid[2] /= a_size;

    // Calculate Centroid B
    b_centroid[0] = 0.0f;
    b_centroid[1] = 0.0f;
    b_centroid[2] = 0.0f;
    for (unsigned int i = 0; i < b_size; i++)
    {
        b_centroid[0] += (*features_b)[i].x;
        b_centroid[1] += (*features_b)[i].y;
        b_centroid[2] += (*features_b)[i].z;
    }
    b_centroid[0] /= b_size;
    b_centroid[1] /= b_size;
    b_centroid[2] /= b_size;

    double htdata[9] = {0.0f};
    double hdata[9] = {0.0f};
    double q1data[3] = {0.0f};
    double q2data[3] = {0.0f};
    CvMat Htotal = cvMat(3,3,CV_64FC1, &htdata[0]); //64F means double
    CvMat H = cvMat(3,3,CV_64FC1, &hdata[0]); //64F means double
    CvMat q1 = cvMat(3,1,CV_64FC1, &q1data[0]);
    CvMat q2 = cvMat(3,1,CV_64FC1, &q2data[0]);	

    // Sum Up q's
    for (unsigned int i = 0; i < a_size; i++)
    {
        q1data[0] = (*features_b)[i].x - b_centroid[0];
        q1data[1] = (*features_b)[i].y - b_centroid[1];
        q1data[2] = (*features_b)[i].z - b_centroid[2];
        q2data[0] = (*features_a)[i].x - a_centroid[0];
        q2data[1] = (*features_a)[i].y - a_centroid[1];
        q2data[2] = (*features_a)[i].z - a_centroid[2];
        
        cvGEMM(&q1, &q2, 1.0, NULL, 0.0, &H, CV_GEMM_B_T);//NULL);               
        
        for (unsigned int s = 0; s < 9; s++) { htdata[s] = htdata[s] + hdata[s]; } // sum
    }

    double utdata[9] = {0};
    double adata[9] = {0};
    double vdata[9] = {0};
    CvMat Ut = cvMat(3,3,CV_64FC1, &utdata[0]); //64F means double
    CvMat A = cvMat(3,3,CV_64FC1, &adata[0]); //64F means double
    CvMat V = cvMat(3,3,CV_64FC1, &vdata[0]); //64F means double
    cvSVD(&Htotal, &A, &Ut, &V, CV_SVD_MODIFY_A | CV_SVD_U_T); // hope this works

    // Find Rotation
    double rdata[9] = {0};
    CvMat rotation = cvMat(3,3,CV_64FC1, &rdata[0]); //64F means double
    cvGEMM(&V, &Ut, 1.0, NULL, 0.0, &rotation, NULL);
    
    // Calculate determinate
    float rotdet =   rdata[0]*rdata[3+1]*rdata[6+2]
                   + rdata[1]*rdata[3+2]*rdata[6+0]
                   + rdata[2]*rdata[3+0]*rdata[6+1]
                   - rdata[0]*rdata[3+2]*rdata[6+1]
                   - rdata[1]*rdata[3+0]*rdata[6+2]
                   - rdata[2]*rdata[3+1]*rdata[6+0];
                
    // If det(x) is -1.0, then there is a reflection due to coplanar points
    if (fabs(rotdet + 1.0f) < 1e-6) {
        printf("REFLECTION IN LEAST SQUARES! \n");
        vdata[0+2] = -vdata[0+2];
        vdata[3+2] = -vdata[3+2];
        vdata[6+2] = -vdata[6+2];
        cvGEMM(&V, &Ut, 1.0, NULL, 0.0, &rotation, NULL);
    }

    // Find Translation
    double translation[3] = {0};
    double datacentRot[3] = {0};
    CvMat centRot = cvMat(3,1,CV_64FC1, &datacentRot[0]); //64F means double
    CvMat newCentMat = cvMat(3,1,CV_64FC1, &b_centroid[0]); //64F means double
    cvGEMM(&rotation, &newCentMat, 1.0, NULL, 0.0, &centRot, NULL);
    translation[0] = a_centroid[0] - datacentRot[0];
    translation[1] = a_centroid[1] - datacentRot[1];
    translation[2] = a_centroid[2] - datacentRot[2];
    
    for (unsigned int i = 0; i < 9; i++) { rotation_data[i] = rdata[i]; }
    for (unsigned int i = 0; i < 3; i++) { trans_data[i] = translation[i]; }
}                             
                             
void VisualOdometry::findClosestPointKD( std::vector<Vector3>* search_features, std::vector<Vector3>* dictionary_features,
                                         std::vector<std::vector<unsigned int> >& ptpairs)
{
    double tt;
    
    // Clear the return vector
    ptpairs.clear();

    // How many matches to get
    int n = 1;
    
	// Build the k-d tree
    tt = (double)cvGetTickCount();
	CvMat dictionary_features_mat = cvMat(dictionary_features->size(), 3, CV_32FC1, &(*dictionary_features)[0].x);
	CvFeatureTree *dictionary_tree = cvCreateKDTree(&dictionary_features_mat);
    tt = (double)cvGetTickCount() - tt;
    printf( "ICP: Create Feature Tree Time = %gms\n", tt/(cvGetTickFrequency()*1000.)); 
        
	// Make the necessary search matrices
	int searchSize = search_features->size();
	CvMat search_features_mat = cvMat(searchSize, 3, CV_32FC1, &(*search_features)[0].x);

	int *matches_data = new int[searchSize*(n+1)];
	double *distance_data = new double[searchSize*(n+1)];
	CvMat matches = cvMat(searchSize, (n+1), CV_32SC1, matches_data);
	CvMat distance = cvMat(searchSize, (n+1), CV_64FC1, distance_data);

	// Run tree search (the 20 is the # of leaves to visit)
	tt = (double)cvGetTickCount();
	cvFindFeatures(dictionary_tree, &search_features_mat, &matches, &distance, (n+1), 20);
    tt = (double)cvGetTickCount() - tt;
    printf( "ICP: Find Features Time = %gms\n", tt/(cvGetTickFrequency()*1000.)); 
    
    //printf("PAIRS : %i vs %i\n", searchSize, (int)(dictionary_features->size()));
    for(int i = 0; i < searchSize; i++ ) // for each search descriptor
    {
    	// (Last element is the best match)
    	
    	//if (CV_MAT_ELEM(distance,double,i,1) < sqrt(0.8)*CV_MAT_ELEM(distance,double,i,0)) {
    	//if (CV_MAT_ELEM(distance,double,i,1) < 0.8*CV_MAT_ELEM(distance,double,i,0)) {
    	if (1) {//ABS check here....???
    		bool found_match = false;
    		std::vector<unsigned int> temp;
	    	temp.push_back(i);
	    	for (int j = n; j > 0; j--) {
    			temp.push_back(CV_MAT_ELEM(matches,int,i,j));
    
                // check for nan
    			if (CV_MAT_ELEM(distance,double,i,0) != CV_MAT_ELEM(distance,double,i,0))
        			printf("distance : %f\n", CV_MAT_ELEM(distance,double,i,0));
        			
    			found_match = CV_MAT_ELEM(distance,double,i,0) < 0.01;
    			if (found_match)
    				break;
    		}
    		if (found_match)
    			ptpairs.push_back(temp);
    	}
    }
    
    cvReleaseFeatureTree(dictionary_tree);
    delete [] matches_data;
    delete [] distance_data;
}

// Inputs: Two Keyframes and Estimates for Rotation and Translation
// Output: Refined Estimates
void VisualOdometry::ICPKeyframes(Keyframe* kA, Keyframe* kB, double* rot_estimate, double* trans_estimate, double* rotation_data, double* trans_data)
{
    /*************************************
       Initialize Before Iteratation
    *************************************/
    double tt;
    std::vector<Vector3> features_a;
    std::vector<Vector3> features_b_pre;
    std::vector<Vector3> features_b;
    double p1[3] = {0}; double p2[3] = {0};
    double rdata[9] = {0}; double translation[3] = {0};
    CvMat P1 = cvMat(3,1,CV_64FC1, &p1[0]); //64F means double
    CvMat P2 = cvMat(3,1,CV_64FC1, &p2[0]); //64F means double
    CvMat R = cvMat(3,3,CV_64FC1, &rdata[0]); //64F means double
    std::vector<std::vector<unsigned int> > ptpairs;
    for (unsigned int i = 0; i < 9; i++) { rdata[i] = rot_estimate[i]; }
    for (unsigned int i = 0; i < 3; i++) { translation[i] = trans_estimate[i]; }
    
    // Use Images To Get Subsample of Points
    IplImage* img_8uc1 = cvCreateImage( cvGetSize(kA->im), 8, 1 );
    IplImage* img_edge = cvCreateImage( cvGetSize(kA->im), 8, 1 );
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* first_contour = NULL;
    
    
    cvNamedWindow( "WTF", 1 );
    IplImage* img_8uc3 = cvCreateImage( cvGetSize(kA->im), 8, 3 );

    
    tt = (double)cvGetTickCount();
    
    first_contour = NULL;
    cvClearMemStorage( storage );
    cvCvtColor( kA->im, img_8uc1, CV_BGR2GRAY );
    cvSet(img_edge, cvScalar(0));
    cvThreshold( img_8uc1, img_edge, 50, 255, CV_THRESH_BINARY );
    int Nc = cvFindContours(
        img_edge,
        storage,
        &first_contour,
        sizeof(CvContour),
        CV_RETR_LIST
    );
    
    cvCvtColor( img_8uc1, img_8uc3, CV_GRAY2BGR );
    CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
    cvDrawContours(
        img_8uc3,
        first_contour,
        color,
        color,
        1,
        // Try different values of max_level, and see what happens
        2,
        8
    );
    cvShowImage( "WTF", img_8uc3 );
    cvWaitKey(10);
    
    for( CvSeq* c = first_contour; c != NULL; c = c->h_next ) 
    {
    
        for( int i = 0; i < c->total; ++i ) 
        {
            CvPoint* p = CV_GET_SEQ_ELEM( CvPoint, c, i );
            unsigned int index = cvRound(p->y)*kA->point_step + cvRound(p->x);
            
            bool isnan = (kA->points[index].x != kA->points[index].x || kA->points[index].y != kA->points[index].y || kA->points[index].z != kA->points[index].z);            
            if ( !isnan && kA->points[index].z > 0.25f )
            {
                Vector3 tempv;
                tempv.x = kA->points[index].x;
                tempv.y = kA->points[index].y;
                tempv.z = kA->points[index].z;
                features_a.push_back(tempv);
            }
        }
    }
 
 cvDestroyWindow( "WTF" );
 
    first_contour = NULL;
    cvClearMemStorage( storage );
    cvCvtColor( kB->im, img_8uc1, CV_BGR2GRAY );
    cvThreshold( img_8uc1, img_edge, 50, 255, CV_THRESH_BINARY );
    Nc = cvFindContours(
        img_edge,
        storage,
        &first_contour,
        sizeof(CvContour),
        CV_RETR_LIST
    );
    for( CvSeq* c = first_contour; c != NULL; c = c->h_next ) 
    {
        for( int i = 0; i < c->total; ++i ) 
        {
            CvPoint* p = CV_GET_SEQ_ELEM( CvPoint, c, i );
            unsigned int index = cvRound(p->y)*kB->point_step + cvRound(p->x);
            
            bool isnan = (kB->points[index].x != kB->points[index].x || kB->points[index].y != kB->points[index].y || kB->points[index].z != kB->points[index].z);            
            if ( !isnan && kB->points[index].z > 0.25f )
            {
                Vector3 tempv;
                tempv.x = kB->points[index].x;
                tempv.y = kB->points[index].y;
                tempv.z = kB->points[index].z;
                features_b_pre.push_back(tempv);
            }
        }
    }
 
    tt = (double)cvGetTickCount() - tt;
    printf( "ICP: Contour time for 2 images = %gms\n", tt/(cvGetTickFrequency()*1000.)); 
 
    // Add each features in first set
    /*for (int i = 0; i < kA->numberOf3DPoints; i++)
    {
        bool isnan = (kA->points[i].x != kA->points[i].x || kA->points[i].y != kA->points[i].y || kA->points[i].z != kA->points[i].z);
        if (!isnan)
        {
            Vector3 tempv;
            tempv.x = kA->points[i].x;
            tempv.y = kA->points[i].y;
            tempv.z = kA->points[i].z;
            features_a.push_back(tempv);
        }
    }

    // Add each features in the second set
    for (int i = 0; i < kB->numberOf3DPoints; i++)
    {
        bool isnan = (kB->points[i].x != kB->points[i].x || kB->points[i].y != kB->points[i].y || kB->points[i].z != kB->points[i].z);
        if (!isnan)
        {
            Vector3 tempv;
            tempv.x = kB->points[i].x;
            tempv.y = kB->points[i].y;
            tempv.z = kB->points[i].z;
            features_b_pre.push_back(tempv);
        }
    }*/
    
    features_b.resize(features_b_pre.size());

    // How many matches to get
    int n = 1;
    
	// Build the k-d tree
    tt = (double)cvGetTickCount();
	CvMat dictionary_features_mat = cvMat(features_a.size(), 3, CV_32FC1, &(features_a[0].x));
	CvFeatureTree *dictionary_tree = cvCreateKDTree(&dictionary_features_mat);
    tt = (double)cvGetTickCount() - tt;
    printf( "ICP: Create Feature Tree Time = %gms\n", tt/(cvGetTickFrequency()*1000.)); 
    
    printf( "ICP: Sizes %i and %i \n", features_a.size(), features_b.size());
    
    // Make the necessary search matrices
	int searchSize = features_b.size();
	CvMat search_features_mat = cvMat(searchSize, 3, CV_32FC1, &(features_b[0].x));

	int *matches_data = new int[searchSize*(n+1)];
	double *distance_data = new double[searchSize*(n+1)];
	CvMat matches = cvMat(searchSize, (n+1), CV_32SC1, matches_data);
	CvMat distance = cvMat(searchSize, (n+1), CV_64FC1, distance_data);
    
    /***************************
           Iterate
    ***************************/
    for (int q = 0; q < 50; q++)
    {
	    tt = (double)cvGetTickCount();
        ptpairs.clear();
        
        // For each feature in the second set
        for (int i = 0; i < features_b_pre.size(); i++)
        {
            // Rotate it into the first frame
            p1[0] = features_b_pre[i].x;
            p1[1] = features_b_pre[i].y;
            p1[2] = features_b_pre[i].z;
            cvGEMM(&R, &P1, 1.0, NULL, 0.0, &P2, NULL);
            features_b[i].x = p2[0] + translation[0];
            features_b[i].y = p2[1] + translation[1];
            features_b[i].z = p2[2] + translation[2];
        }
        
        // Search for closest partner
        // Run tree search (the 20 is the # of leaves to visit)
	    cvFindFeatures(dictionary_tree, &search_features_mat, &matches, &distance, (n+1), 20);
        
        for(int i = 0; i < searchSize; i++ ) // for each search descriptor
        {
        	// (Last element is the best match)
        	
        	//if (CV_MAT_ELEM(distance,double,i,1) < sqrt(0.8)*CV_MAT_ELEM(distance,double,i,0)) {
        	//if (CV_MAT_ELEM(distance,double,i,1) < 0.8*CV_MAT_ELEM(distance,double,i,0)) {
        	if (1) {//ABS check here....???
        		bool found_match = false;
        		std::vector<unsigned int> temp;
	        	temp.push_back(i);
	        	for (int j = n; j > 0; j--) {
        			temp.push_back(CV_MAT_ELEM(matches,int,i,j));
        
                    // check for nan
        			if (CV_MAT_ELEM(distance,double,i,0) != CV_MAT_ELEM(distance,double,i,0))
            			printf("distance : %f\n", CV_MAT_ELEM(distance,double,i,0));
            			
        			found_match = CV_MAT_ELEM(distance,double,i,0) < 0.01;
        			if (found_match)
        				break;
        		}
        		if (found_match)
        			ptpairs.push_back(temp);
        	}
        }
        
        // Add pair to list for least squares
        std::vector<Vector3> features_a2;
        features_a2.resize(ptpairs.size());
        std::vector<Vector3> features_b2;
        features_b2.resize(ptpairs.size());
        for (int i = 0; i < ptpairs.size(); i++)
        {
            features_a2[i] = features_a[ptpairs[i][1]]; // 1.. is dictionary
            features_b2[i] = features_b[ptpairs[i][0]]; // 0 is search
        }
        printf("There was %i matched points for ICP\n", ptpairs.size());

        // Least Squares
        ArunLeastSquares(&features_a2, &features_b2, rotation_data, trans_data);
        
        // Check if change is so little that iterations should end early
        // TODO
        
        double global[16] = {0};
        double newfix[16] = {0};
        CvMat globalMat = cvMat(4,4,CV_64FC1, &global[0]); //64F means double precision
        CvMat newfixMat = cvMat(4,4,CV_64FC1, &newfix[0]); //64F means double precision
        
        global[0]  = rdata[0];     global[1]  = rdata[1];     global[2]  = rdata[2];     global[3]  = translation[0];
        global[4]  = rdata[3];     global[5]  = rdata[4];     global[6]  = rdata[5];     global[7]  = translation[1];
        global[8]  = rdata[6];     global[9]  = rdata[7];     global[10] = rdata[8];     global[11] = translation[2];
        global[12] = 0.0f;         global[13] = 0.0f;         global[14] = 0.0f;         global[15] = 1.0f;
        
        newfix[0]  = rotation_data[0];     newfix[1]  = rotation_data[1];     newfix[2]  = rotation_data[2];     newfix[3]  = trans_data[0];
        newfix[4]  = rotation_data[3];     newfix[5]  = rotation_data[4];     newfix[6]  = rotation_data[5];     newfix[7]  = trans_data[1];
        newfix[8]  = rotation_data[6];     newfix[9]  = rotation_data[7];     newfix[10] = rotation_data[8];     newfix[11] = trans_data[2];
        newfix[12] = 0.0f;                 newfix[13] = 0.0f;                 newfix[14] = 0.0f;                 newfix[15] = 1.0f;
        
        cvGEMM(&globalMat, &newfixMat, 1.0, NULL, 0.0, &globalMat, NULL);
        
        rdata[0] = global[0];     rdata[1] = global[1];     rdata[2] = global[2];     translation[0] = global[3];
        rdata[3] = global[4];     rdata[4] = global[5];     rdata[5] = global[6];     translation[1] = global[7];
        rdata[6] = global[8];     rdata[7] = global[9];     rdata[8] = global[10];    translation[2] = global[11];
        
        tt = (double)cvGetTickCount() - tt;
        printf( "ICP Iteration Time = %gms\n", tt/(cvGetTickFrequency()*1000.)); 
    }
    
    for (unsigned int i = 0; i < 9; i++) { rotation_data[i] = rdata[i]; }
    for (unsigned int i = 0; i < 3; i++) { trans_data[i] = translation[i]; }
    
    cvReleaseMemStorage(&storage);
    cvReleaseFeatureTree(dictionary_tree);
    cvReleaseImage( &img_8uc1 );
    cvReleaseImage( &img_edge );
    delete [] matches_data;
    delete [] distance_data;
}
                    
                             
                             
                             
