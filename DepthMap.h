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

using namespace std;

void EpipolarMatching(CameraPose input, IplImage* Image1, IplImage* Image2, IplImage* colorImage1, IplImage* colorImage2);
int SSD (const int x, const int  y, const int cpx, const int cpy);
IplImage* plot_Stereo_imagesf(IplImage *IGray, IplImage *IGray1, int NumPts, const vector<cv::Point> pt, const vector<cv::Point> cpt);
float MatchingProcess(int* pixelRef, IplImage*Image2, float cpx, float cpy, int Pathsize);
void ReferencePatch(IplImage* Image1, int Size, int* pixelRef, const int x, const int y);
cv::Mat DepthSmooth(cv::Mat depthMap,IplImage* colorImage);

#endif /* defined(__OpenCVtracking__DepthMap__) */
