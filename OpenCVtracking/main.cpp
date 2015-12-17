
//
//  main.cpp
//  OpenCVtracking
//
//  Created by C-HChang on 5/1/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#include <iostream>
#include "F_matrix.h"
#include "videoprocessing.h"
#include "matchingandtracking.h"
#include "miscul.h"
#include "OpenglPlot.h"
//#include "FeaturePoint.h"
//#include "Relative_Pose.h"
#include "RunSFM_Main.hpp"
#include "Map3DStruct.h"
#include "DepthMap.h"

CvCapture *camCapture;
IplImage* skipNFrames(CvCapture* capture, int n);

using namespace std;
using namespace cv;

int Img_width;
int Img_height;
char key;
bool readfromvideo= 1 ;

int main (int argc, const char * argv[])
{
    if (readfromvideo)
    {

        //Img_width=320;
        //Img_height=240;
        Img_width=640;
        Img_height=480;
    }
    else
    {
        //Img_width=320;
        //Img_height=240;
        Img_width=640;
        Img_height=480;
    }

    if(readfromvideo)
    camCapture = cvCaptureFromFile( "/Users/c-hchang/Desktop/OpenCVtracking/video/P36.mov" );
    else
    camCapture = cvCaptureFromCAM(CV_CAP_ANY);

    if (!(camCapture))
    {
        cout << "Failed to capture from camera" << endl;
        return -1;
    }

    cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_WIDTH,  Img_width);
    cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_HEIGHT, Img_height);

    cout << "Camera opened successfully" << endl;

    IplImage *cameraFrame;

    bool _1stframe=true;
    //bool _1sttrack=true;

    cameraFrame = cvCreateImage(cvSize (Img_width,Img_height), IPL_DEPTH_8U,3);

    IplImage * imgA=0;
    IplImage * imgB=0;
    IplImage * imgC=0;

    IplImage * imgGrayA=0;
    IplImage * imgGrayB=0;

    //Img_width=  320 ;
    //Img_height= 240 ;

    imgA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgC = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);

    imgGrayA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);
    imgGrayB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);

    //IplImage* Two_image = 0;

    vector<Point2f> tempCorners;

    IplImage* frame;

    VideoProcessing VideoProcessing (Img_width, Img_height, readfromvideo);
    CameraPose CameraPose;
    OpenGLPlot  OpenGLPlot (Img_width*2, Img_height*2);
    FeaturePts FeaturePts;
    FeaMapPoints FeaMapPoints;
    _3DPt _3DPt;
    int FrameNum =0;

    /// flow control parameters
    int firstcapture = 1;
    bool SkipthisFrame=0;
    bool ThirdFrame=0;
    int loop =0;
    //  flow control
    do
        if (camCapture)
        {
            int fps = ( int )cvGetCaptureProperty( camCapture, CV_CAP_PROP_FPS );
            if ( !camCapture )
            {
                fprintf( stderr, "Cannot open AVI!\n" );
                return 1;
            }

            // exit if it reaches the last frame
           if( ! cvGrabFrame(camCapture))
            {
               break;
             }

            if ( _1stframe)
            {
                frame = cvQueryFrame(camCapture);
                frame = skipNFrames(camCapture, 2);
                imgB  = cvCloneImage(frame);
            }

            if( ! _1stframe)
            {
                bool CaptureFrames=0;
                if (! SkipthisFrame)
                {
                    frame = skipNFrames(camCapture, 1);
                    frame = VideoProcessing.CaptureFrame(camCapture);
                    if (VideoProcessing.EndOfFrame)
                         goto stop;

                    if (! CaptureFrames)
                    {
                        if (VideoProcessing.captureNextFrame)
                        {
                            CaptureFrames = 1;
                        }
                    }
                }
            else
            {
                cout<<"Frame skipped "<<endl;
                frame = VideoProcessing.CaptureFrame(camCapture);
                if (VideoProcessing.EndOfFrame)
                    goto stop;

                if (! CaptureFrames)
                {
                    if (VideoProcessing.captureNextFrame)
                    {
                        CaptureFrames =1;
                    }
                }

            }

                cvShowImage("test", frame);
                cvWaitKey(1);

                if(CaptureFrames==1)
                {
                    imgA= frame;  // new frame //
                    imgC= cvCloneImage(imgB);  // previous frame //

                    cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);
                    cvCvtColor(imgC,imgGrayB, CV_BGR2GRAY);

                    std::vector<CvPoint2D32f> match_query;
                    std::vector<CvPoint2D32f> match_train;

                    //LKFeatures LKFeatures (imgGrayA,imgGrayB, LKFeatures. BRIEF_descriptor);
                    LKFeatures LKFeatures (imgGrayA,imgGrayB, LKFeatures. Freak_descriptor);
                    LKFeatures.FeaturesMatched (match_query, match_train);

                    //SIFTfeature SIFTfeature(imgGrayA, imgGrayB,2, 0.05);
                    //SIFTfeature.SIFTfeaturematch(match_query, match_train);

                    //SURFfeature  SURFfeature(imgGrayA, imgGrayB,  400 );
                    //SURFfeature. SURFfeaturematch(match_query, match_train);

                    //FAST_ FAST_(70, imgGrayA, imgGrayB, FAST_. SURF_descriptor);
                    //FAST_. FAST_tracking(match_query, match_train);

                    //ORBfeature  ORBfeature(imgGrayA, imgGrayB,0.1,0.1);
                    //ORBfeature. ORBfeaturematch(match_query, match_train);

                    int size_match= (int) match_query.size();
                    int numTrialFmatrix = 30;
                    int numTrialRelativePose = 20;
                    int Focuslength= 300;
                    int Ransac_threshold= 2.0;
                    float MaxAngle;
                    float F_matrix_threshold=0.8;

                    if(readfromvideo)
                    MaxAngle = 0.03;
                    else  
                    MaxAngle = 0.03;

                    EpipolarGeometry EpipolarGeometry(match_query, match_train, size_match, numTrialFmatrix, numTrialRelativePose, Focuslength, Ransac_threshold,F_matrix_threshold,Img_width,Img_height );

                    EpipolarGeometry.FindFundamentalMatrix();
                    EpipolarGeometry.FindRelativePose(EpipolarGeometry. FivePoints );
                    EpipolarGeometry.FindApicalAngle(MaxAngle);

                   //cout<<"Apical angle : "<<EpipolarGeometry.ApicalAngle<<endl;
                    

                    if  (EpipolarGeometry.SkipFrame())
                    {
                        SkipthisFrame =1;
                    }
                    else
                    {
                        vector<v3_t> V3Dpts1;
                        if(! ThirdFrame)
                        {
                            vector<v2_t> left_pts;
                            vector<v2_t> right_pts;
                            vector<v3_t> V3Dpts;
                            SkipthisFrame =0;

                            EpipolarGeometry.InitializeFirstPmatrix();
                            EpipolarGeometry.TwoviewTriangulation(left_pts,right_pts,V3Dpts);

                            CameraPose.InitializeFirstTwoKMatrix(EpipolarGeometry.K1matrix, EpipolarGeometry.K2matrix);

                            CameraPose.First2viewInitialization(EpipolarGeometry.R1matrix, EpipolarGeometry.R_relative, EpipolarGeometry.t1matrix, EpipolarGeometry.t_relative);


                            FrameNum = 1 ;
                            //FeaturePts.LoadFeatureList(FrameNum);

                            _3DPt.Initial((int)V3Dpts.size(), V3Dpts, left_pts, right_pts, FrameNum);

                            _3DPt.ColorIniitalization(frame , right_pts);

                            EpipolarMatching(CameraPose, imgGrayA, imgGrayB, imgA, imgC);

                        }
//  // start from 3rd frames //
                        else
                        {

                            CameraPose. InitializeKMatrix(Focuslength);
                             // Show the current frame number for feature map //
                             // need to add frame number increased in here //
                            FrameNum += 1;  // Show the current frame number for feature map //  // change to current frame //

                            // Connect feature point and create feature tracks

                            vector<v2_t>Reproject2D;
                            vector<v3_t>Project3D;


                            _3DPt.ConnectNewFrame( EpipolarGeometry.num_ofrefined_pts, EpipolarGeometry.lrefined_pt /* current left point*/ , EpipolarGeometry.rrefined_pt /*current new frame points*/ , FrameNum,  Reproject2D, Project3D);

                            cout<<"corresponding points : "<<Reproject2D.size()<<" "<<Project3D.size()<<endl;
                            
                            CameraPose.Egomotion(EpipolarGeometry.R_relative, EpipolarGeometry.t_relative, Project3D, Reproject2D);

                            vector<vector<v2_t> >V2Location;
                            vector<vector<int> >V2Frame;
                            vector<int> Overlap;


                            _3DPt. _3DPtGeneration(EpipolarGeometry.num_ofrefined_pts, FrameNum,  EpipolarGeometry.lrefined_pt, EpipolarGeometry.rrefined_pt, V2Location, V2Frame, Overlap);


                            vector<v3_t> Tempv3Dpts;
                            vector<bool> tempvector;

                            CameraPose.Triangulation_N_frame_Map(0,
                                                                 0,
                                                                 0,
                                                             V2Location /*2D points location*/ ,
                                                             V2Frame    /*frame number*/,
                                                             Tempv3Dpts /*triangulation output*/,
                                                             tempvector /*save array for refinement*/);

                            _3DPt.PointRefinement(Tempv3Dpts, V2Location, V2Frame, Overlap , tempvector);

                            vector<v3_t> _3DPoints;
                            vector<int> SelectedIndex;

                            int NumCamera=3;  /* this is used to generate # of camera for bundle adjustement*/
                            _3DPt.MapGeneration(_3DPoints , V2Location,  V2Frame, SelectedIndex, FrameNum, NumCamera);

                            
                            int StartCmaera= (FrameNum - NumCamera)+1;
                            vector<size_t> RemoveIdx;

                            double error = RunSFM_Nviews_Main ((int)_3DPoints.size() /*number of 3D pts */,
                                                                                          3,
                                                                                          StartCmaera,
                                                                                          CameraPose. mRcMatrix, /*camera rotation matrix*/
                                                                                          CameraPose. mTcMatrix,  /*camera translation matrix*/
                                                                                          CameraPose. KMatrix,  /*camera instrinstic matrix*/
                                                                                          V2Location,
                                                                                          V2Frame    /*frame number*/,
                                                                                          _3DPoints       /*triangulation output*/,
                                                                                          SelectedIndex,
                                                                                          RemoveIdx);

                            _3DPt.MapUpdate( SelectedIndex, _3DPoints , RemoveIdx, imgA);

                            loop++;
                        }
                          imgB= cvCloneImage(frame);
                         ThirdFrame= true;
                   }

                }


                firstcapture=0;
            }
                // exit if user presses 'x'          
                _1stframe=false;
    }

    while (true) ;

    cvReleaseImage(&frame);
    cvReleaseImage(&imgA);
    cvReleaseImage(&imgB);
    cvReleaseImage(&imgC);
    cvReleaseImage(&imgGrayA);
    cvReleaseImage(&imgGrayB);


    stop:
      cout<<"jump to stop : generate 3D ply file ";
      vector<v3_t> output;
      vector<v3_t> color;
      for (int i=0;i<_3DPt.Numof3Dpts();i++){
           output.push_back( _3DPt.Read3DPoint(i));
           color.push_back( _3DPt.ReadColor(i));
           //cout<<
           //int Rcolor =  _3DPt._3D[i].R; int Gcolor =  _3DPt._3D[i].G; int Bcolor =  _3DPt._3D[i].B;
          //cout << Rcolor <<" "<<Gcolor<<" "<<Bcolor<<endl;
      }
    cout<<endl;
      for(int i=0;i< CameraPose.mTcMatrix.size();i++) {
        double T[3];
        memcpy(T, CameraPose.mTcMatrix[i].n,3*sizeof(double));
        cout<<T[0]<<" "<<1*T[1]<<" "<<T[2]<<" "<<255<<" "<<0<<" "<<0<<endl;
      }

     DumpPointsToPly("/Users/c-hchang/Desktop/Opencvtracking/Point/result.ply", output ,_3DPt.Numof3Dpts(), color);
     //DumpPointsToPly("/Users/c-hchang/Desktop/Opencvtracking/Point/result.ply", output , _3DPt.Numof3Dpts());
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
