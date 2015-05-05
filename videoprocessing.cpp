//
//  videoprocessing.cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/8/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "videoprocessing.h"

IplImage* VideoProcessing :: GrayImg(IplImage* image)
{
    IplImage*  imgG  = cvCreateImage(cv::Size(ImgWidth, ImgHeight), IPL_DEPTH_8U, 1);
    cvCvtColor(image,imgG, CV_BGR2GRAY);
    
    cout<<"test "<<endl;
    return(NULL);
}

IplImage*  VideoProcessing :: ToGrayImg()
{
    IplImage* imgGray  = cvCreateImage(cv::Size(ImgWidth, ImgHeight), IPL_DEPTH_8U, 1);
    cvCvtColor(VideoProcessing::Image1,imgGray, CV_BGR2GRAY);
    return(imgGray);
}

IplImage* VideoProcessing :: capture(CvCapture* camCapture)
{
     frame = cvQueryFrame(camCapture);
     Image1=cvCloneImage(frame);
    

    return(Image1);

}
bool VideoProcessing :: CaptureNextframe()
{
    return(captureNextFrame)  ;
}

VideoProcessing:: VideoProcessing (int Width , int Height)
{
    ImgWidth = Width;
    ImgHeight = Height;
    count_frame= FALSE;
    
     imgGray1  = cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 1);
     imgGray2  = cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 1);
    
     Image1=  cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 3);
     Image2=  cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 3);
     //Image3=  cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 3);
     //frame=  cvCreateImage(cv::Size(ImgWidth, ImgHeight), IPL_DEPTH_8U, 3);;
     
     captureNextFrame =FALSE;
     captureThirdFrame= FALSE;
     
}

IplImage* VideoProcessing :: CaptureInitialFrame(CvCapture* camCapture)
{
    
    
    vector<cv::Point2f> cornerfirst, cornersecond;

    if (count_frame)
    {   
       
        frame = cvQueryFrame(camCapture);
        
        Image2= cvCloneImage(frame);
        
        cvCvtColor(Image1,imgGray1, CV_BGR2GRAY);  
        cvCvtColor(Image2,imgGray2, CV_BGR2GRAY);
       
        goodFeaturesToTrack(imgGray1, cornerfirst,  400, 0.001, 16);
        goodFeaturesToTrack(imgGray2, cornersecond, 400, 0.001, 16);
        
        cout<< cornerfirst.size()<<" "<<cornersecond.size()<<endl;
    
    if ((int) cornerfirst.size() >= cornersecond.size())
    { 
        frame = Image1;
        cout<<"captured first frame"<<endl;
   
    }
    else
    {
        frame = Image2;
        cout<<"captured second frame"<<endl;
    }
    
        count_frame= FALSE;
        captureNextFrame=true;
        
    }

    if( ! count_frame)
    {
       
        VideoProcessing::frame = cvQueryFrame(camCapture);
        VideoProcessing::Image1= cvCloneImage(frame);
        count_frame = true;  
    }

    //count_frame++;
    //countFrameReset= true ;
    
    return (frame);
}
VideoProcessing::~VideoProcessing()
{
    cvReleaseImage(&Image1);
    cvReleaseImage(&Image2);
    cvReleaseImage(&imgGray1);
    cvReleaseImage(&imgGray2);
    cvReleaseImage(&frame);

    
}