// File for PCD file export of ic2020 data

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "ic2020_vodom/keyframe.h"
#include "Keyframe.h"
#include "KeyframeUpdater.h"

#define OUTFILE "map.hawk"

void copyInMat(float * mat, Keyframe * k)
{
  mat[0] = k->rotation[0];
  mat[4] = k->rotation[1];
  mat[8]  = k->rotation[2];
  mat[12] = k->translation[0];
  mat[1] = k->rotation[3];
  mat[5] = k->rotation[4];
  mat[9]  = k->rotation[5];
  mat[13] = k->translation[1];
  mat[2] = k->rotation[6];
  mat[6] = k->rotation[7];
  mat[10] = k->rotation[8];
  mat[14] = k->translation[2];
  mat[3] = 0.0f;
  mat[7] = 0.0f;
  mat[11] = 0.0f;
  mat[15] = 1.0f;
}

void vecMatMul(float * x, const float * mat)
{
  float x1 = x[0];
  float x2 = x[1];
  float x3 = x[2];
  
  x[0] = mat[0]*x1 + mat[4]*x2 + mat[8]*x3 + mat[12];
  x[1] = mat[1]*x1 + mat[5]*x2 + mat[9]*x3 + mat[13];
  x[2] = mat[2]*x1 + mat[6]*x2 + mat[10]*x3 + mat[14];
}

void matMatMul(float * m1, float * m2)
{
  float destMat[16];
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4;j++) {
      destMat[4*j + i] = 0;
      for(int k = 0; k < 4; k++) {
	destMat[4*j + i] += m1[4*i+k]*m2[4*k+j];
      }
    }
  }
  memcpy(m1, destMat, 16*sizeof(float));
}

void exportCloud(std::vector<Keyframe*> * _keyframes)
{
  std::vector<Keyframe*> keyframes = *_keyframes;
  
  if(keyframes.size() == 0) {
    printf("No data added to map, skipping cloud dump.\n");
    return;
  }
  
  // Integrated position matrix
  float idmat[16];
  idmat[0] = 1.0f;
  idmat[1] = 0.0f;
  idmat[2] = 0.0f;
  idmat[3] = 0.0f;
  idmat[4] = 0.0f;
  idmat[5] = 1.0f;
  idmat[6] = 0.0f;
  idmat[7] = 0.0f;
  idmat[8] = 0.0f;
  idmat[9] = 0.0f;
  idmat[10] = 1.0f;
  idmat[11] = 0.0f;
  idmat[12] = 0.0f;
  idmat[13] = 0.0f;
  idmat[14] = 0.0f;
  idmat[15] = 1.0f;

  float mat[16];
  memcpy(idmat, mat, 16*sizeof(float));

  // Open destination file
  FILE * f = fopen(OUTFILE, "w");
  
  if(f == NULL) {
    printf("Error: unable to open output file %s\n", OUTFILE);
    return;
  }

  int pointCount = 0;

  for(int i = 0; i < keyframes.size(); i++) {
    if(keyframes[i]->global == true) {
      memcpy(idmat, mat, 16*sizeof(float));
    }

    float keyMat[16];
    copyInMat(keyMat, keyframes[i]);
    matMatMul(mat, keyMat);
    
    for(int j = 0; j < keyframes[i]->numberOf3DPoints; j++) {
      vecMatMul((float *)&(keyframes[i]->points[j]), mat);
    }

    fwrite((void *)&(keyframes[i]->points[0]), 4*sizeof(float), 640*480, f);

    pointCount += keyframes[i]->numberOf3DPoints;
  }

  fclose(f);

  printf("Wrote %d points to: %d\n", pointCount, OUTFILE);
  
}

