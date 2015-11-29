
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

        Img_width=320;
        Img_height=240;
    }
    else
    {
        Img_width=320;
        Img_height=240;
    }

    if(readfromvideo)
    camCapture = cvCaptureFromFile( "/Users/c-hchang/Desktop/OpenCVtracking/video/P6.mp4" );
    else
    camCapture = cvCaptureFromCAM(CV_CAP_ANY);

    if (!(camCapture))
    {
        cout << "Failed to capture from camera" << endl;
        return 0;
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
                frame = skipNFrames(camCapture, 10);
                imgB  = cvCloneImage(frame);
            }

            if( ! _1stframe)
            {
                bool CaptureFrames=0;
                if (! SkipthisFrame)
                {
                    frame = skipNFrames(camCapture, 0);
                    frame = VideoProcessing.CaptureFrame(camCapture);

                    if (! CaptureFrames)
                    {
                        if (VideoProcessing.captureNextFrame)
                        {
                            CaptureFrames =1;
                        }
                    }
                }
            else
            {
                cout<<"Frame skipped "<<endl;
                frame = VideoProcessing.CaptureFrame(camCapture);

                if (! CaptureFrames)
                {
                    if (VideoProcessing.captureNextFrame)
                    {
                        CaptureFrames =1;
                    }
                }

            }

                if(CaptureFrames==1)
                {
                    imgA= frame;  // new frame //
                    imgC= cvCloneImage(imgB);  // previous frame //

                    cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);
                    cvCvtColor(imgC,imgGrayB, CV_BGR2GRAY);

                    std::vector<CvPoint2D32f> match_query;
                    std::vector<CvPoint2D32f> match_train;

                    LKFeatures LKFeatures (imgGrayA,imgGrayB, LKFeatures. BRIEF_descriptor);
                    LKFeatures.FeaturesMatched (match_query, match_train);

                    //SIFTfeature SIFTfeature(imgGrayA, imgGrayB,2, 0.05);
                    //SIFTfeature.SIFTfeaturematch(match_query, match_train);

                    //SURFfeature  SURFfeature(imgGrayA, imgGrayB,  400 );
                    //SURFfeature. SURFfeaturematch(match_query, match_train);

                    //FAST_ FAST_(2, imgGrayA, imgGrayB, FAST_. SURF_descriptor);
                    //FAST_. FAST_tracking(match_query, match_train);

                    //ORBfeature  ORBfeature(imgGrayA, imgGrayB,0.01,0.01);
                    //ORBfeature. ORBfeaturematch(match_query, match_train);

                    int size_match= (int) match_query.size();
                    int numTrialFmatrix = 30;
                    int numTrialRelativePose = 20;
                    int Focuslength= 300;
                    int Ransac_threshold= 2.0;
                    float MaxAngle;
                    float F_matrix_threshold=1;

                    if(readfromvideo)
                    MaxAngle = 0.035;
                    else  
                    MaxAngle = 0.035;

                    EpipolarGeometry EpipolarGeometry(match_query, match_train, size_match, numTrialFmatrix, numTrialRelativePose, Focuslength, Ransac_threshold,F_matrix_threshold,Img_width,Img_height );

                    EpipolarGeometry.FindFundamentalMatrix();
                    EpipolarGeometry.FindRelativePose(EpipolarGeometry. FivePoints );
                    EpipolarGeometry.FindApicalAngle(MaxAngle);
                    cout<<"Apical angle : "<<EpipolarGeometry.ApicalAngle<<endl;
                    
                    IplImage* Two_image=EpipolarGeometry.plot_two_imagesf(imgA, imgC);
                    cvShowImage("test", Two_image);
                    cvReleaseImage(&Two_image);

                    //cvWaitKey( 5 );

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

                            FeaturePts.Loadv2Pts( left_pts, right_pts);
                            FeaturePts.Loadv3Pts(V3Dpts);

                            FrameNum = 1 ;
                            FeaturePts.LoadFeatureList(FrameNum);

                            _3DPt.Initial((int)V3Dpts.size(), V3Dpts, left_pts, right_pts, FrameNum);


                            FeaMapPoints.LoadFMv2Pts( left_pts, right_pts);
                            FeaMapPoints.LoadFMv3Pts(V3Dpts);
                            FeaMapPoints.LoadFMFeatureList(FrameNum);

                        }
  // start from 3rd frames //
                        else
                        {

                            CameraPose. InitializeKMatrix(Focuslength);
                             // Show the current frame number for feature map //
                             // need to add frame number increased in here //
                            FrameNum += 1;  // Show the current frame number for feature map //  // change to current frame //

                            // Connect feature point and create feature tracks

                            //FeaturePts.ConnectedVideoSequence( FeaturePts.m_rightPts, EpipolarGeometry.lrefined_pt /*connected pts*/ , EpipolarGeometry.rrefined_pt  /* current pts*/ , EpipolarGeometry.num_ofrefined_pts,FrameNum);

                            vector<v2_t>Reproject2D;
                            vector<v3_t>Project3D;


                            _3DPt.ConnectNewFrame( EpipolarGeometry.num_ofrefined_pts, EpipolarGeometry.lrefined_pt /* current left point*/ , EpipolarGeometry.rrefined_pt /*current new frame points*/ , FrameNum,  Reproject2D, Project3D);

                            
                            CameraPose.Egomotion(EpipolarGeometry.R_relative, EpipolarGeometry.t_relative, Project3D, Reproject2D);

                            vector<vector<v2_t> >V2Location;
                            vector<vector<int> >V2Frame;
                            vector<int> Overlap;

                            cout<<"v2 location: "<<V2Location.size()<<" "<<V2Frame.size()<<endl;

                            _3DPt. _3DPtGeneration(EpipolarGeometry.num_ofrefined_pts, FrameNum,  EpipolarGeometry.lrefined_pt, EpipolarGeometry.rrefined_pt, V2Location, V2Frame, Overlap);


                            cout<<"before Map: "<<endl;
                            for (int i=0;i< V2Frame.size();i++){
                                for  (int j=0;j< V2Frame[i].size();j++){
                                    cout<<V2Frame[i][j]<<" ";
                                }
                                cout<<endl;
                            }


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


                            cout<<"after Map: "<<endl;
                            for (int i=0;i< V2Frame.size();i++){
                                for  (int j=0;j< V2Frame[i].size();j++){
                                    cout<<V2Frame[i][j]<<" ";
                                }
                                cout<<endl;
                            }


                             cout<< "Number of points before SFM "<<(int) FeaturePts.m_3Dpts.size()<<endl;
                            

//                            double error = RunSFM_Nviews_Main (FeaturePts. m_3Dpts.size() /*number of 3D pts */,
//                                                              3,
//                                                              0,
//                                                              CameraPose. mtriRotmatrix, /*camera rotation matrix*/
//                                                              CameraPose. mtriTcmatrix,  /*camera translation matrix*/
//                                                              CameraPose. mtriKmatrix,  /*camera instrinstic matrix*/
//                                                              FeaturePts. mv2_location  /*2D points location*/ ,
//                                                              FeaturePts. mv2_frame     /*frame number*/,
//                                                              FeaturePts. m_3Dpts       /*triangulation output*/);

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

                            //cout<<"after Map: "<<endl;
                            //for (int i=0;i< V2Frame.size();i++){
                            //    for  (int j=0;j< V2Frame[i].size();j++){
                            //        cout<<V2Frame[i][j]<<" ";
                            //    }
                            //    cout<<endl;
                            //}



                            _3DPt.MapUpdate(SelectedIndex, _3DPoints , RemoveIdx);

            
                            //cout<< FeaturePts.mv2_frame.size()<<endl;
                            //CameraPose.RemoveTri();  // remove third

                            //vector<v2_t> left_pts;
                            //vector<v2_t> right_pts;
                            //vector<v3_t> V3Dpts;
                            loop++;

                            if (loop >5)
                            {
                                for(int i=0;i< CameraPose.mTcMatrix.size();i++) {
                                    double T[3];
                                    memcpy(T, CameraPose.mTcMatrix[i].n,3*sizeof(double));
                                    cout<<T[0]<<" "<<-1*T[1]<<" "<<T[2]<<endl;
                                    //matrix_print(1,3,T);
                                }

                                DumpPointsToPly("/Users/c-hchang/Desktop/Opencvtracking/Point/result.ply", FeaturePts. _3DLocation , FeaturePts. _3DLocation.size());
                                break;
                            }

                            //FeaturePts.UpdatedFeatureTrack( left_pts, right_pts, V3Dpts, FrameNum );

                            //FeaturePts.CleanFeatureTrack();

                            //cout<<"Number of points loaded into "<<left_pts.size()<<endl;
                            
                            //FeaturePts. Loadv2Pts(left_pts, right_pts);

                            //FeaturePts. Loadv3Pts(V3Dpts);

                            //FeaturePts. LoadFeatureList(FrameNum);

                            //double T[3];
                            //double T1[3];

                            //double Rtemp[9];
                            //memcpy(Rtemp, CameraPose.mtriRotmatrix[FrameNum].n,9*sizeof(double));

                            //memcpy(T, CameraPose.mtriTcmatrix[FrameNum].n,3*sizeof(double));

                            //matrix_product331( Rtemp, T , T1);

                            //T1[0]=T[0]; T1[1]=-T[1];T1[2]=T[2];
                            //matrix_print(3,1,T);


                            //OpenGLPlot. Setview(FeaturePts. _3DLocation);

                            //OpenGLPlot. PlotCamera(T);

                            //OpenGLPlot. PlotVertex(EpipolarGeometry.NumofPts(), V3Dpts);
                            //OpenGLPlot.PlotVertex((int) V3Dpts.size() , V3Dpts);
                            //cout<< CameraPose.SizeofPose()<<endl;
                            //cout<< FeaturePts.NumReproject<<endl;


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
