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
 * <ic2020_vodom/src/VisualOdometry.h>
 * 
 * revision 1.0
 */
 
 #ifndef H_VIS_ODOM
#define H_VIS_ODOM

#include <vector>
#include "cv.h"
#include "SURFHelper.h"
#include "Vector3.h"

/////////////////////////
// Variable Parameters //
/////////////////////////

#define RANSAC_3D_THRESH 0.05f //m
#define RANSAC_THRESH_ERROR_PER_M 0.04f //0.03f
#define RANSAC_PERCENT_PASS 0.5f //percent
#define RANSAC_MAX_ITER 150

#define RANSAC_REPROJ_THRESH 6.0f

class VisualOdometry
{
	public:

		VisualOdometry();
		virtual ~VisualOdometry();
        
        static bool RANSAC6DReproj (std::vector<feature>* features_a, std::vector<feature>* features_b, 
                                    std::vector<unsigned int>* pairs, std::vector<unsigned int>* filtered_pairs);
        
        static bool RANSAC6D       (std::vector<feature>* features_a, std::vector<feature>* features_b, 
                                    std::vector<unsigned int>* pairs, std::vector<unsigned int>* filtered_pairs,
                                    feature* corn_a, feature* corn_b, char* corn_pairs, unsigned int num_corn_pairs,
                                    std::vector<unsigned int>* filtered_corn_pairs);

        static bool RANSAC6DFast   (std::vector<feature>* features_a, std::vector<feature>* features_b, 
                                    std::vector<unsigned int>* pairs, std::vector<unsigned int>* filtered_pairs,
                                    feature* corn_a, feature* corn_b, char* corn_pairs, unsigned int num_corn_pairs,
                                    std::vector<unsigned int>* filtered_corn_pairs,
                                    float imwid, float imhei, unsigned int xbins, unsigned int ybins, unsigned int maxperbin);

        static bool RANSAC6DFastCorn(feature* corn_a, feature* corn_b, char* corn_pairs, unsigned int num_corn_pairs,
                                     std::vector<unsigned int>* filtered_corn_pairs,
                                     float imwid, float imhei, unsigned int xbins, unsigned int ybins, unsigned int maxperbin);

        static void ArunLeastSquares2D(std::vector<feature>* features_a, std::vector<feature>* features_b,
                                       double* rotation_data, double* trans_data);
        static void ArunLeastSquares(std::vector<feature>* features_a, std::vector<feature>* features_b,
                                     double* rotation_data, double* trans_data);
        static void ArunLeastSquares(std::vector<Vector3>* features_a, std::vector<Vector3>* features_b,
                                     double* rotation_data, double* trans_data);
                                     
        static void RotateAndTransf (float* orig_point, float* rot, float* trans, float* new_point);
        static void RotateAndTrans (double* orig_point, double* rot, double* trans, double* new_point);
        
        static void findClosestPointKD( std::vector<Vector3>* search_features, std::vector<Vector3>* dictionary_features,
                                        std::vector<std::vector<unsigned int> >& ptpairs);
        static void ICPKeyframes(Keyframe* kA, Keyframe* kB, double* rot_estimate, double* trans_estimate, double* rotation_data, double* trans_data);

	private:
        static bool GetRotFromRnd3(std::vector<feature>* features_a, std::vector<feature>* features_b, double* rot, double* trans);
};

#endif
