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
 * <ic2020_vodom/src/SURFHelper.cpp>
 * 
 * revision 1.0
 */
 
#include "SURFHelper.h"

#include <stdio.h>
#include <iostream> 

using namespace std;

// Calculates squared error between descriptors
double SURFHelper::compareSURFDescriptors(const float* d1, const float* d2, double best, int length)
{
    double total_cost = 0;
    //assert( length % 4 == 0 ); //TODO some kind of check
    for( int i = 0; i < length; i += 4 )
    {
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}

// Input is a single search_feature and a vector of dictionary features
// Output is the index of the closest partner, -1 means no best partner
int SURFHelper::naiveSURFNearestNeighbor(surfdesc* search_feature, int search_laplacian, std::vector<surfdesc>* dictionary_features)
{
    int length = 64;//(int)(dict_descriptors->elem_size/sizeof(float));
    int neighbor = -1;
    double d, dist1 = 1e6, dist2 = 1e6;

    for(unsigned int i = 0; i < dictionary_features->size(); i++ ) // for each desc
    {
        //if( search_laplacian != kp->laplacian ) // check keypoint laplacians for speed up
          //  continue;
        d = compareSURFDescriptors(&search_feature->desc[0], &(*dictionary_features)[i].desc[0], dist2, length );
        if( d < dist1 )
        {
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if (dist1 < 0.8*dist2) {
        return neighbor;
    }

    return -1;
}

// Input is a list of search features and dictionary features
// Output is the vector of pairings:   ptpairs[a1, a2, b1, b2, c1, c2...etc]
void SURFHelper::findSURFPairs( std::vector<surfdesc>* search_features, std::vector<surfdesc>* dictionary_features, std::vector<unsigned int>& ptpairs)
{
    ptpairs.clear();

    for(unsigned int i = 0; i < search_features->size(); i++ ) // for each search descriptor
    {
        int nearest_neighbor = naiveSURFNearestNeighbor( &(*search_features)[i], 0, dictionary_features);
        if (nearest_neighbor != -1)
        {
            ptpairs.push_back(i);
            ptpairs.push_back(nearest_neighbor);
        }
    }
}

// Input is a list of search features and dictionary features
// Output is the vector of pairings:   ptpairs[a1, a2, b1, b2, c1, c2...etc]
void SURFHelper::findSURFPairsKD( std::vector<surfdesc>* search_features, std::vector<surfdesc>* dictionary_features, std::vector<unsigned int>& ptpairs)
{
    ptpairs.clear();
	// Build the k-d tree
	
	CvMat dictionary_features_mat = cvMat(dictionary_features->size(), 64, CV_32FC1, (*dictionary_features)[0].desc);
	CvFeatureTree *dictionary_tree = cvCreateKDTree(&dictionary_features_mat);
	
	// Make the necessary search matrices
	int searchSize = search_features->size();
	cout << "search size: " << searchSize << " dictionary: " << dictionary_features->size() << "\n\n";
	
	CvMat search_features_mat = cvMat(searchSize, 64, CV_32FC1, (*search_features)[0].desc);
	
	int *matches_data = new int[searchSize*2];
	double *distance_data = new double[searchSize*2];
	CvMat matches = cvMat(searchSize, 2, CV_32SC1, matches_data);
	CvMat distance = cvMat(searchSize, 2, CV_64FC1, distance_data);
	
	// Run tree search (the 20 is the # of leaves to visit)
	//cvFindFeatures(dictionary_tree, &search_features_mat, &matches, &distance, 2, 200);
	cvFindFeatures(dictionary_tree, &search_features_mat, &matches, &distance, 2, 20);
	
    //printf("PAIRS : %i vs %i\n", searchSize, (int)(dictionary_features->size()));
    for(int i = 0; i < searchSize; i++ ) // for each search descriptor
    {
    	// (Last element is the best match)
    	
    	printf("#: %i KD dists: %f : %f ratio: %f, match: %i %i, raw: %i %i\n", i,
    			CV_MAT_ELEM(distance,double,i,1), CV_MAT_ELEM(distance,double,i,0),
    					CV_MAT_ELEM(distance,double,i,1)/(CV_MAT_ELEM(distance,double,i,0)*sqrt(0.8)),
    					CV_MAT_ELEM(matches,int,i,1), CV_MAT_ELEM(matches,int,i,0),
    					matches_data[i*2+1], matches_data[i*2]);
    	
    	//if (CV_MAT_ELEM(distance,double,i,1) < sqrt(0.8)*CV_MAT_ELEM(distance,double,i,0)) {
    	if (CV_MAT_ELEM(distance,double,i,1) < 0.8*CV_MAT_ELEM(distance,double,i,0)) {
	    	ptpairs.push_back(i);
    		ptpairs.push_back(CV_MAT_ELEM(matches,int,i,1));
    		//printf("Link %i to %i\n", i, CV_MAT_ELEM(matches,int,i,1));
    	}
    }
    cvReleaseFeatureTree(dictionary_tree);
    delete [] matches_data;
    delete [] distance_data;
}

void SURFHelper::findNSURFPairsKD( std::vector<surfdesc>* search_features, std::vector<surfdesc>* dictionary_features, std::vector<std::vector<unsigned int> >& ptpairs, int n)
{
    ptpairs.clear();
	// Build the k-d tree
	
	CvMat dictionary_features_mat = cvMat(dictionary_features->size(), 64, CV_32FC1, (*dictionary_features)[0].desc);
	CvFeatureTree *dictionary_tree = cvCreateKDTree(&dictionary_features_mat);
	
	// Make the necessary search matrices
	int searchSize = search_features->size();
	
	CvMat search_features_mat = cvMat(searchSize, 64, CV_32FC1, (*search_features)[0].desc);
	
	int *matches_data = new int[searchSize*(n+1)];
	double *distance_data = new double[searchSize*(n+1)];
	CvMat matches = cvMat(searchSize, (n+1), CV_32SC1, matches_data);
	CvMat distance = cvMat(searchSize, (n+1), CV_64FC1, distance_data);
	
	// Run tree search (the 20 is the # of leaves to visit)
	cvFindFeatures(dictionary_tree, &search_features_mat, &matches, &distance, (n+1), 20);
	
    //printf("PAIRS : %i vs %i\n", searchSize, (int)(dictionary_features->size()));
    for(int i = 0; i < searchSize; i++ ) // for each search descriptor
    {
    	// (Last element is the best match)
    	
    	/*
    	printf("#: %i KD dists: %f : %f ratio: %f, match: %i %i, raw: %i %i\n", i,
    			CV_MAT_ELEM(distance,double,i,1), CV_MAT_ELEM(distance,double,i,0),
    					CV_MAT_ELEM(distance,double,i,1)/(CV_MAT_ELEM(distance,double,i,0)*sqrt(0.8)),
    					CV_MAT_ELEM(matches,int,i,1), CV_MAT_ELEM(matches,int,i,0),
    					matches_data[i*2+1], matches_data[i*2]);
    	*/
    	
    	//if (CV_MAT_ELEM(distance,double,i,1) < sqrt(0.8)*CV_MAT_ELEM(distance,double,i,0)) {
    	//if (CV_MAT_ELEM(distance,double,i,1) < 0.8*CV_MAT_ELEM(distance,double,i,0)) {
    	
    	if (1) {//ABS check here....???
    		bool found_match = false;
    		vector<unsigned int> temp;
	    	temp.push_back(i);
	    	for (int j = n; j > 0; j--) {
    			temp.push_back(CV_MAT_ELEM(matches,int,i,j));
//    			found_match = CV_MAT_ELEM(distance,double,j,1) < sqrt(0.8)*CV_MAT_ELEM(distance,double,i,j-1);
    			found_match = CV_MAT_ELEM(distance,double,i,j) < sqrt(0.8)*CV_MAT_ELEM(distance,double,i,j-1);
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


