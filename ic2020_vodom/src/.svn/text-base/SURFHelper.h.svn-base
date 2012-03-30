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
 * <ic2020_vodom/src/SURFHelper.h>
 * 
 * revision 1.0
 */
 
 #ifndef H_SURF_HELPER
#define H_SURF_HELPER

#include <vector>
#include "cv.h"

#include "Keyframe.h"

typedef struct pos2D_ {
    float f1;
    float f2;
} pos2D;

typedef struct pos3D_ {
    float f1;
    float f2;
    float f3;
} pos3D;

class SURFHelper
{
	public:

		SURFHelper();
		virtual ~SURFHelper();

        static void findSURFPairs(std::vector<surfdesc>* search_features, std::vector<surfdesc>* dictionary_features, std::vector<unsigned int>& ptpairs); // Returned pairs are [search, dictionary]
		static void findSURFPairsKD(std::vector<surfdesc>* search_features, std::vector<surfdesc>* dictionary_features, std::vector<unsigned int>& ptpairs);
		static void findNSURFPairsKD(std::vector<surfdesc>* search_features, std::vector<surfdesc>* dictionary_features, std::vector<std::vector<unsigned int> >& ptpairs, int n); // Returned are <[search, dic1, dic2, ... up to n dics]>
		
	private:
    	static double compareSURFDescriptors(const float* d1, const float* d2, double best, int length);
        static int naiveSURFNearestNeighbor(surfdesc* search_feature, int search_laplacian, std::vector<surfdesc>* dictionary_features);
};

#endif
