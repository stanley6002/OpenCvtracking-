//
//  matchingandtracking.cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/9/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "matchingandtracking.h"


using namespace std;
using namespace cv;

 FAST_ :: FAST_(int level, IplImage* imgGrayA, IplImage* imgGrayB, Function Descriptor)
{
    
    FAST_::Level= level;    

    ImageGray1 = cv::cvarrToMat(imgGrayA);
    
    ImageGray2 = cv::cvarrToMat(imgGrayB);
    
    if (Descriptor == SURF_descriptor)
    { 
        Surf_activate=1;
    }
    else 
    {
        Surf_activate=0;
    
    }
}

void FAST_ :: FAST_tracking(std::vector<CvPoint2D32f>& match_query, std::vector<CvPoint2D32f>& match_train)
 {

     if(! Surf_activate)
     {
         cv::FAST(ImageGray1,  FAST_query_kpts,  Level, TRUE);
         cv::FAST(ImageGray2,  FAST_train_kpts,  Level, TRUE);

         
         FAST_descriptor = new cv::BriefDescriptorExtractor(64);
         
         FAST_descriptor->compute(ImageGray1,  FAST_query_kpts, FAST_query_desc);
         FAST_descriptor->compute(ImageGray2,  FAST_train_kpts,FAST_train_desc);
       
         FAST_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
     
         std::vector<cv::KeyPoint> test_kpts;
         FAST_H_prev = cv::Mat::eye(3,3,CV_32FC1);
         
         //warpKeypoints(FAST_H_prev.inv(), FAST_query_kpts, test_kpts);
         //cv::Mat FAST_mask = windowedMatchingMask(test_kpts, FAST_train_kpts, 100, 100 );
         //FAST_matcher->match(FAST_query_desc, FAST_train_desc, FAST_matche, FAST_mask);
            int i=0;
         
         for (; i< FAST_matche.size(); i++)
         //for (; i< FAST_matcher.size(); i++)
         {
             int queryIdx = FAST_matche[i].queryIdx;
             int trainIdx = FAST_matche[i].trainIdx;
             
             //int queryIdx = FAST_matcher[i].queryIdx;
             //int trainIdx = FAST_matcher[i].trainIdx;
             
             match_query.push_back(FAST_query_kpts[queryIdx].pt);
             match_train.push_back(FAST_train_kpts[trainIdx].pt);
             
         }
     }
     
     if(Surf_activate)
     {
         cv::FAST(ImageGray1,  FAST_query_kpts,  Level, TRUE);
         cv::FAST(ImageGray2,  FAST_train_kpts,  Level, TRUE);



         FAST_descriptor=new cv::OrbDescriptorExtractor(4,2);
         
         FAST_descriptor->compute(ImageGray1, FAST_query_kpts, FAST_query_desc);
         FAST_descriptor->compute(ImageGray2, FAST_train_kpts, FAST_train_desc);

        std::vector<cv::DMatch> FAST_matcher;
        
        std::vector<cv::KeyPoint> test_kpts;
         
         //FAST_H_prev = cv::Mat::eye(3,3,CV_32FC1);
         //warpKeypoints(FAST_H_prev.inv(), FAST_query_kpts, test_kpts);
         //cv::Mat FAST_mask = windowedMatchingMask(test_kpts, FAST_train_kpts, 250, 250);
        
         //new cv::SiftFeatureDetector();
         //detector = new SiftFeatureDetector;
         //cv::DescriptorExtractor
         
         cv::BruteForceMatcher<cv::L2<float> > matcher;
         matcher.match(FAST_query_desc, FAST_train_desc, FAST_matcher);
         
         //std::vector<std::vector<cv::DMatch> > matches;
         
         int i=0;
         for (; i< FAST_matcher.size(); i++)
          {

             int queryIdx = FAST_matcher[i].queryIdx;
             int trainIdx = FAST_matcher[i].trainIdx;
             
             match_query.push_back(FAST_query_kpts[queryIdx].pt);
             match_train.push_back(FAST_train_kpts[trainIdx].pt);
         
           }
     }
} 

 FAST_::~ FAST_()
{
    //cvReleaseImage(&ImageGray1);
    //cvReleaseImage(&ImageGray2);
    ImageGray1.release();
    ImageGray2.release();
}

void FAST_ :: warpKeypoints(const cv::Mat& H, const vector<cv::KeyPoint>& in, vector<cv::KeyPoint>& out)
{
    vector<cv::Point2f> pts;
    keypoints2points(in, pts);
    vector<cv::Point2f> pts_w(pts.size());
    cv::Mat m_pts_w(pts_w);
    
    
    perspectiveTransform(cv::Mat(pts), m_pts_w, H);
    
    
    points2keypoints(pts_w, out);
}

void FAST_ :: keypoints2points(const vector<cv::KeyPoint>& in, vector<cv::Point2f>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(in[i].pt);
    }
}

void FAST_ :: points2keypoints(const vector<cv::Point2f>& in, vector<cv::KeyPoint>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        
        out.push_back(cv::KeyPoint(in[i], 1));
    }
}

LKFeatures :: LKFeatures(IplImage* imgGrayA, IplImage* imgGrayB, LKFeatures::Function input)
{

  int Numberofcorner=400;
  int threshold1=0.01;
  int Windowsize=7;
  int Wsize =16;
  ParametersInitialized(Numberofcorner, threshold1, Wsize, input);

   ImageGray1 = cv::cvarrToMat(imgGrayA);

   ImageGray2 = cv::cvarrToMat(imgGrayB);

  //ImageGray2 = cvCloneImage(imgGrayB);
  LKFeaturesTracking();    

}
void LKFeatures:: ParametersInitialized (int Numberofcorner, int threshold1, int Wsize, LKFeatures::Function input)
{
      NumberCorn = Numberofcorner;
      Threshold  = threshold1;
      W_size = Wsize;
      Windowsize =7;
    
    if(input == Optical_flow)
    {
        UseOptical_flow= true;
     }
    else
    {
        UseOptical_flow= false;
    }    
           
}

void LKFeatures::  LKFeaturesTracking ()
{
  
    
   if (UseOptical_flow)
   {
       
       goodFeaturesToTrack(ImageGray1, corners,  400, 0.001,9);
       cornerSubPix( ImageGray1, corners, cv::Size(7,7) , cv::Size(-1,-1) , cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.001 ));
       calcOpticalFlowPyrLK(ImageGray1, ImageGray2, corners, nextPts, status, err, cv::Size(45,30));
   }
   else
   {
       goodFeaturesToTrack(ImageGray1,  LK_query_kpts, 400,0.0001,8);
       //cornerSubPix( ImageGray1, LK_query_kpts, cv::Size(7,7) , cv::Size(-1,-1) , cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.001 ));
       goodFeaturesToTrack(ImageGray2,  LK_train_kpts, 400,0.0001,8);
       //cornerSubPix( ImageGray2, LK_train_kpts, cv::Size(7,7) , cv::Size(-1,-1) , cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.001 ));
       //calcOpticalFlowPyrLK(ImageGray1, ImageGray2, LK_query_kpts, LK_train_kpts, status, err, cv::Size(30,30));

       LK_descriptor = new cv::BriefDescriptorExtractor(16);

       //// CONVERT 2D pointf to keypoint

       int size_= (int) LK_query_kpts.size();
       std::vector<cv::KeyPoint> temp_query;
       //temp_query.resize(size_);

       std::vector<cv::KeyPoint> temp_train;
       //temp_train.resize(size_);

       std::vector<cv::KeyPoint> query_kpts;
       cv::KeyPoint tempx;


       for(int i=0;i<size_;i++)

       {
          temp_query.push_back(cv::KeyPoint(LK_query_kpts[i], 1.f));
          temp_train.push_back(cv::KeyPoint(LK_train_kpts[i], 1.f));
           //cout<<temp_train[i].pt.x<<" "<<temp_train[i].pt.y<<endl;
       }


       cout<<temp_train.size()<<endl;



       LK_descriptor->compute(ImageGray1,  temp_query,  LK_query_desc);
       LK_descriptor->compute(ImageGray2,  temp_train,  LK_train_desc);


       cout<<LK_query_desc.size()<<endl;

       cv::BFMatcher matcher(cv::NORM_HAMMING, true);
       matcher.match(LK_query_desc, LK_train_desc, LK_matche);

       for (int i=0; i< (int) LK_matche.size(); i++)
       {

           int queryIdx = LK_matche[i].queryIdx;
           int trainIdx = LK_matche[i].trainIdx;

           cv::Point2f pt_q;
           cv::Point2f pt_t;

           pt_q= (cv::Point2f) temp_query[queryIdx].pt;
           pt_t= (cv::Point2f) temp_train[trainIdx].pt;

           corners.push_back(pt_q);
           nextPts.push_back(pt_t);

       }
   }
}
void LKFeatures ::  FeaturesMatched  (std::vector<CvPoint2D32f> &match_query, std::vector<CvPoint2D32f> &match_train)
{
    int size = (int) corners.size();
    for (int i=0;i< size; i++)
    {
        CvPoint2D32f pt_q;
        CvPoint2D32f pt_t;
       pt_q.x = corners[i].x;
       pt_q.y = corners[i].y; 
       pt_t.x = nextPts[i].x;
       pt_t.y = nextPts[i].y;
        
        match_query.push_back(pt_q);
        match_train.push_back(pt_t);
    }  
}
LKFeatures:: ~LKFeatures()
{  
   ImageGray1.release();
   ImageGray2.release();
}

SIFTfeature::SIFTfeature(IplImage* imgGrayA, IplImage* imgGrayB,float Th1, float Th2)
{

    ImageGray1 = cv::cvarrToMat(imgGrayA);
    ImageGray2 = cv::cvarrToMat(imgGrayB);

    th1=Th1;
    th2=Th2;
}

void SIFTfeature::SIFTfeaturematch(vector<CvPoint2D32f> &match_query, vector<CvPoint2D32f> &match_train)
{
    std::vector<cv::KeyPoint> kpts;
    std::vector<cv::KeyPoint> Quepts;

    cv::Mat desc;
    cv::Mat src;

    Ptr<FeatureDetector> siftdet;
    siftdet = new cv::SiftFeatureDetector(th1,th2) ;

    Ptr<DescriptorExtractor> Extractor;
    Extractor= new FREAK(true, true);
    siftdet->detect(ImageGray1, kpts);
    Extractor-> compute(ImageGray1, kpts, desc);
    siftdet->detect(ImageGray2, Quepts);
    Extractor-> compute(ImageGray2, Quepts, src);

    BFMatcher matcher(NORM_HAMMING, true);
    std::vector<cv::DMatch> vec_matches;
    matcher.match(desc, src, vec_matches);


    for (size_t i = 0; i < vec_matches.size(); ++i)
    {
        const DMatch& match = vec_matches[i];
        CvPoint2D32f pointA;
        CvPoint2D32f pointB;
        pointA.x=kpts[match.queryIdx].pt.x;
        pointA.y=kpts[match.queryIdx].pt.y;

        pointB.x=Quepts[match.trainIdx].pt.x;
        pointB.y=Quepts[match.trainIdx].pt.y;

        match_query.push_back(pointA);
        match_train.push_back(pointB);
    }

    desc.release();
    src.release();

}

SIFTfeature:: ~SIFTfeature()
{
    ImageGray1.release();
    ImageGray2.release();
}


