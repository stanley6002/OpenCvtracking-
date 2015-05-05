//
//  main.cpp
//  OpenCVtracking
//
//  Created by C-HChang on 5/1/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#include "vector.h"
#include <iostream>
#include "F_matrix.h"
#include "videoprocessing.h"
#include "matchingandtracking.h"

CvCapture *camCapture;
IplImage* skipNFrames(CvCapture* capture, int n);

using namespace std;
using namespace cv;

int Img_width = 320;
int Img_height= 240;
char key;


int main (int argc, const char * argv[])
{

    int key = 0;


    // Initialize camera and OpenCV image
    //CvCapture* capture = cvCaptureFromCAM( 0 );
    CvCapture* capture = cvCaptureFromFile( "/Users/c-hchang/Desktop/OpenCVtracking/video/P1.mov" );
    IplImage* frame = cvQueryFrame( capture );

    // Check
    if ( !capture )
    {
        fprintf( stderr, "Cannot open AVI!\n" );
        return 1;
    }

    // Get the fps, needed to set the delay
    int fps = ( int )cvGetCaptureProperty( capture, CV_CAP_PROP_FPS );

    // Create a window to display the video
    cvNamedWindow( "video", CV_WINDOW_AUTOSIZE );

    while( key != 'x' )
    {
        // get the image frame
        frame = cvQueryFrame( capture );

        // exit if unsuccessful
        if( !frame ) break;

        // display current frame
        cvShowImage( "video", frame );

        // exit if user presses 'x'
        key = cvWaitKey( 1000 / fps );
    }

    // Tidy up
    cvDestroyWindow( "video" );
    cvReleaseCapture( &capture );
    
    return 0;
}

IplImage* skipNFrames(CvCapture* capture, int n)
{
    for(int i = 0; i < n; ++i)
    {
        if(cvQueryFrame(capture) == NULL)
        {
            return NULL;
        }
    }

    return cvQueryFrame(capture);
}
