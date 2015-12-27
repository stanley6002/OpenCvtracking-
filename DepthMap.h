//
//  DepthMap.h
//  OpenCVtracking
//
//  Created by C-HChang on 12/10/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#ifndef __OpenCVtracking__DepthMap__
#define __OpenCVtracking__DepthMap__

#include <stdio.h>
# include <vector>
# include <iostream>
# include "matrix.h"
# include "vector.h"
# include "Relative_Pose.h"
#include "eigen3/Eigen/Dense"

using namespace std;

typedef  struct
{
    cv::Mat depthMap ;
    //unsigned int * Dxy;
    cv::Mat Dxy;
} DepthData;


class DMap
{
    public:
          void Initial_DepthData(int ImgWidth, int ImgHeight);
          DMap(int ImgWidth, int ImgHeight);
          vector<DepthData>DMapVector;
          bool checkAvailable(int x, int y);
};


//void EpipolarMatching(CameraPose input, IplImage* Image1, IplImage* Image2, IplImage* colorImage1, IplImage* colorImage2);

void EpipolarMatching (DMap& DMap ,CameraPose input, IplImage* Image1, IplImage* Image2, IplImage* colorImage1, IplImage* colorImage2);

int SSD (const int x, const int  y, const int cpx, const int cpy);

IplImage* plot_Stereo_imagesf(IplImage *IGray, IplImage *IGray1, int NumPts, const vector<cv::Point> pt, const vector<cv::Point> cpt);

float MatchingProcess(int* pixelRef, IplImage*Image2, float cpx, float cpy, int Pathsize);

void ReferencePatch(IplImage* Image1, int Size, int* pixelRef, const int x, const int y);

void  DepthSmooth(cv::Mat depthMap,IplImage* colorImage);

void DepthRecovery(int ImgCenterWidth, int ImgCenterHeight,  int cpx, int cpy , Eigen::Vector3d Tmatrix /*absolute pose*/, cv::Mat DepthMap,  Eigen::Vector3d& pInf);


bool Match_Main(IplImage* Image1, IplImage* Image2 , int x , int y , Eigen::Vector3d& pInf ,float MaxIdepth, float MinIdepth ,
                Eigen::MatrixXd KMat, Eigen::MatrixXd RMat, Eigen::Vector3d TMat, int& cpxfinal, int& cpyfinal , float& incx , float& incy, float& MinError);

void EpipolarMatching_1 (double* R_relative, double* T_relative, double* Kmatrix , double* Tmatrix,  IplImage* Image1, IplImage* Image2, IplImage* colorImage1, IplImage* colorImage2, int frameIndex);

void DepthSmooth_1 (cv::Mat depthMap ,IplImage* colorImage, double* Tmatrix, int frameIndex);

void ExportDepthData(IplImage* colorImage , cv::Mat depthMap, int frameIndex, double*Tmatrix);

#endif /* defined(__OpenCVtracking__DepthMap__) */
