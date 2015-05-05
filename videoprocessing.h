//
//  videoprocessing.h
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/8/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "Opencvheaders.h"

class VideoProcessing
{
   
public:
   
    int ImgWidth ;
    int ImgHeight;
    
    bool captureNextFrame;
    bool captureThirdFrame;
    
    IplImage* imgGray1;
    IplImage* imgGray2;
    
    IplImage* Image1;
    IplImage* Image2;
    
    IplImage* frame; 
    
    VideoProcessing (int Height , int Width);
    
    IplImage*  CaptureInitialFrame(CvCapture* camCapture);
    bool CaptureNextframe();
    
    ~ VideoProcessing ();
    IplImage* capture(CvCapture* camCapture);
    
    IplImage* GrayImg(IplImage* image);
    
    IplImage* ToGrayImg();
    
private:
 
    bool count_frame; 
    
     
    //bool countFrameRest;
};