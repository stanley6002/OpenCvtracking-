//
//  DepthMap.cpp
//  OpenCVtracking
//
//  Created by C-HChang on 12/10/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#include "DepthMap.h"

#include <fstream>
#include <stdlib.h>
#include <vector>

#define MIN_WIDTH  30
#define MAX_WIDTH  30
#define MIN_HEIGHT 30
#define MAX_HEIGHT 30
#define MAXGRA 3000
//#define SSDWindow 9
#define MAX_ERROR 1000000.0f
#define RegSize 15
#define SSDWindow 25

#define NAVA   99999
#define UPDATE 11111

// match first two initial frame//
void EpipolarMatching (DMap& DMap ,CameraPose input, IplImage* Image1, IplImage* Image2, IplImage* colorImage1, IplImage* colorImage2)
{
    int ImgWidth = Image1->width;
    int ImgHeight = Image1->height;

    int ImgCenterWidth=  (ImgWidth*0.5);
    int ImgCenterHeight= (ImgHeight*0.5);

    cv::Mat DepthMap = cv::Mat::zeros(ImgHeight,ImgWidth,CV_64F);


    double Kmatrix[9], Rmatrix[9], Tmatrix[3];

    input.PopKMattix(1, Kmatrix);
    input.PopTcMatrix(1, Tmatrix);
    input.PopRotcMatrix(1, Rmatrix);

    Eigen::Vector3d TMat;  // the host matrix for x
    memcpy(TMat.data(), Tmatrix, sizeof(double)*3*1);
    TMat[2]=-1*TMat[2];


    //Eigen::Vector3d TRelative;
    //memcpy(TRelative.data(), Trelative , sizeof(double)*3*1);


    Eigen::MatrixXd RotT(3, 3); // the host matrix for X
    memcpy(RotT.data(), Rmatrix, sizeof(double)*9);
    Eigen::MatrixXd RMat = RotT.transpose();


    //Eigen::MatrixXd RRelativeT(3, 3); // the host matrix for X
    //memcpy(RRelativeT.data(), Rrelative, sizeof(double)*9);
    //Eigen::MatrixXd RRelativeMat = RRelativeT.transpose();


    Eigen::MatrixXd KMat(3, 3); // the host matrix for X
    memcpy(KMat.data(), Kmatrix, sizeof(double)*9);
    KMat(0,2)= 1*ImgWidth*0.5f;
    KMat(1,2)= 1*ImgHeight*0.5f;

    //float pixelx , pixely, pixelDy , pixelDx;

    IplImage* imgB;
    imgB  = cvCloneImage(Image1);
    IplImage* imgC;
    imgC  = cvCloneImage(Image2);

    vector<cv::Point> pt;
    vector<cv::Point> cpt;

    for(int y= MIN_HEIGHT ;y< ImgHeight-MAX_HEIGHT; y++)

        for(int x = MIN_WIDTH ;x<ImgWidth- MAX_WIDTH ;x++){


        // search depth and finding corresponding points  //
        float  MaxIdepth= 0.001f;    // set MaxDepth 100;
        float  MinIdepth= 0.5f;     // set MinDepth 5;
        float  incx=0.0f;
        float  incy=0.0f;
        int    cpxfinal=0;
        int    cpyfinal=0;
        float  MinError =  99999999.0f;

        Eigen::Vector3d pInf;

        bool goodPoint = Match_Main(Image1,Image2 ,  x ,  y , pInf , MaxIdepth,  MinIdepth , KMat, RMat, TMat,  cpxfinal,  cpyfinal ,  incx , incy, MinError);

            if (goodPoint) {

                 //if(test==1001){
                 //    cvCircle(imgC,cv::Point(cpxfinal,cpyfinal), 2.0 , CV_RGB(255,255,255),3.0,8,0);
                 //    cvCircle(imgB,cv::Point(x,y), 0.5 , CV_RGB(255,255,255),3.0,8,0);
                 //   cvShowImage("new1", imgC);
                 //   cvShowImage("new", imgB);
                 //   cvWaitKey();
                 //}
                 //test++;

                if (MinError < MAX_ERROR)
                {

                    if(incx*incx>incy*incy)
                        DepthRecovery (ImgCenterWidth, ImgCenterHeight, cpxfinal, cpyfinal , TMat /*absolute pose*/, DepthMap, pInf);

                    else {
                        Eigen::Vector3d KPnew =  Eigen::Vector3d(cpxfinal,cpyfinal,1.0f);
                        Eigen::Vector3f Xnew= KPnew.cast<float>()-pInf.cast<float>();
                        Xnew = Xnew* (1.0/pInf[2]);
                        double x_3d = (double(cpxfinal)-ImgCenterWidth);
                        double y_3d = (double(cpyfinal)-ImgCenterHeight);
                        Eigen::Vector3f Tk;
                       Tk[0]= 300.0f*TMat[0];
                       Tk[1]= 300.0f*TMat[1];
                       float depth = -1*Tk[2]/Xnew[2];
                       cout << -1*depth*(x_3d/300.0f)<<" "<<-1*depth*(y_3d/300.0f)<<" "<<depth<<endl;
                    }
                 }
              }
     }

    DepthSmooth(DepthMap, colorImage2);

    DMap.DMapVector[0].depthMap= DepthMap;

    for (int i=0;i< ImgWidth ;i++){
            for (int j=0;j< ImgHeight;j++){
                  if(DepthMap.at<double>(j,i)==0.0)
                  {
                      DMap.DMapVector[0].Dxy.at<unsigned int>(j,i) = NAVA;
                  }
            }
    }


    /*
    FILE *file_ = fopen("/Users/c-hchang/Desktop/OpenCVtracking/matrix.txt", "w");
    for (int i = 0; i < ImgWidth ; i++)
    {
        for (int j =0 ;j< ImgHeight ;j++) {
             // Output the vertex //
            fprintf(file_, "%0.0f %s", DepthMap.at<double>(j,i)," ");
        }
        fprintf(file_,"\n");
    }

    fclose(file_);
   */

}
void DepthRecovery(int ImgCenterWidth, int ImgCenterHeight,  int cpx, int cpy , Eigen::Vector3d Tmatrix /*absolute pose*/, cv::Mat DepthMap,  Eigen::Vector3d& pInf)
{


    Eigen::Vector3d KPnew =  Eigen::Vector3d(cpx,cpy,1.0f);

    //cvCircle(imgB,cv::Point(x,y), 0.5 , CV_RGB(255,255,255),3.0,8,0);
    //cvCircle(imgC,cv::Point(cpxfinal,cpyfinal), 0.5 , CV_RGB(255,255,255),3.0,8,0);

    KPnew[0]= KPnew[0]- ImgCenterWidth;
    KPnew[1]= KPnew[1]- ImgCenterHeight;

    pInf[0]= pInf[0] - ImgCenterWidth;
    pInf[1]= pInf[1] - ImgCenterHeight;

    Eigen::Vector3f Xnew = KPnew.cast<float>() -  pInf.cast<float>();

    Eigen::Vector3f Tk;
    Tk[0]=  300.0f*Tmatrix[0];
    Tk[1]=  300.0f*Tmatrix[1];

    float depth = (-1*Tk[0]/Xnew[0])*(1.0/pInf[2]);

    double x_3d = (double(cpx)-ImgCenterWidth);
    double y_3d = (double(cpy)-ImgCenterHeight);

    if(abs(depth) < 40 && depth<0) {
        // cvCircle(imgC,cv::Point( cpxfinal, cpyfinal), 0.5 , CV_RGB(255,255,255),3.0,8,0);
        // cvCircle(imgB,cv::Point( x, y), 0.5 , CV_RGB(255,255,255),3.0,8,0);\
        //cout<<"before : "<< -1*depth*(x_3d/300.0f)<<" "<<-1*depth*(y_3d/300.0f)<<" "<<depth<<endl;
        DepthMap.at<double>(y_3d+ImgCenterHeight, x_3d+ImgCenterWidth)=fabs(depth);
    }

}

bool Match_Main(IplImage* Image1, IplImage* Image2 , int x , int y , Eigen::Vector3d& pInf ,float MaxIdepth, float MinIdepth ,
                Eigen::MatrixXd KMat, Eigen::MatrixXd RMat, Eigen::Vector3d TMat, int& cpxfinal, int& cpyfinal , float& incx , float& incy, float& MinError)
{
    bool UseblePt=0;

    int ImgWidth =  Image1->width;
    int ImgHeight = Image1->height;

    int pixelx = CV_IMAGE_ELEM (Image1, uchar, y,  x+1)  -  CV_IMAGE_ELEM (Image1, uchar, y,  x-1);
    int pixely = CV_IMAGE_ELEM (Image1, uchar, y-1,  x)  -  CV_IMAGE_ELEM (Image1, uchar, y+1,  x);
    int pixelDy= CV_IMAGE_ELEM (Image1, uchar, y-1,  x-1) - CV_IMAGE_ELEM (Image1, uchar, y+1,  x+1);
    int pixelDx= CV_IMAGE_ELEM (Image1, uchar, y+1,  x-1) - CV_IMAGE_ELEM (Image1, uchar, y-1,  x+1);

    float eplGradSquared = (pixelx * pixelx) + (pixely * pixely) + (pixelDy *pixelDy) + (pixelDx*pixelDx) ;

    // search depth and finding corresponding points  //
    // float  MaxIdepth= 0.001f;    // set MaxDepth 100;
    // float  MinIdepth= 0.5f;     // set MinDepth 5;

    if (eplGradSquared > MAXGRA)
    {
        UseblePt=1;

        Eigen::Vector3d KinvP =  Eigen::Vector3d(x,y,1.0f);
        pInf =   KMat*RMat*KMat.inverse()*KinvP;

        Eigen::Vector3f ClosePt = pInf.cast<float>() + KMat.cast<float>()  * TMat.cast<float>() * MinIdepth;
        Eigen::Vector3f FarPt =  pInf.cast<float>()  + KMat.cast<float>()  * TMat.cast<float>() * MaxIdepth;

        ClosePt = ClosePt / ClosePt[2];
        FarPt = FarPt/FarPt[2];

         incx = ClosePt[0] - FarPt[0];
         incy = ClosePt[1] - FarPt[1];

        float eplLength = sqrt(incx*incx+incy*incy);

        incx *= 1.0f/eplLength;
        incy *= 1.0f/eplLength;

        float cpx =  FarPt[0];
        float cpy =  FarPt[1];

        int  counter=0;
        int  pixelRef[SSDWindow];

        //cout<<cpx<<" "<<cpy<<endl;

        ReferencePatch(Image1, SSDWindow /*size*/,  pixelRef, x, y);

        //float SSDError;
        cpxfinal=0;
        cpyfinal=0;

        MinError =  99999999.0;


        while (((cpx) >1 && (cpy)>1 && (cpx)<ImgWidth-1 && (cpy)<ImgHeight-1) && counter< 15 )  // need new boundry test//
        {

            float SSDError=0;

            //int pixelx= CV_IMAGE_ELEM (Image2, uchar, int(cpy),   int(cpx+1)) - CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx-1));
            //int pixely= CV_IMAGE_ELEM (Image2, uchar, int(cpy-1), int(cpx)) - CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx));
            //int pixelDy= CV_IMAGE_ELEM (Image2, uchar,int(cpy-1),  int(cpx-1)) - CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx+1));
            //int pixelDx= CV_IMAGE_ELEM (Image2, uchar,int(cpy+1),  int(cpx-1)) - CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx+1));
            //float eplGradSquared = (pixelx * pixelx) + (pixely * pixely) + (pixelDy*pixelDy)+ (pixelDx*pixelDx);

            SSDError = MatchingProcess(pixelRef, Image2,  cpx,  cpy, SSDWindow);

            if(SSDError<MinError) {
                cpxfinal = cpx;
                cpyfinal = cpy;
                MinError = SSDError;
            }

            cpx+= incx;
            cpy+= incy;
            counter++;
        }
    }

    return(UseblePt);

}
void  DepthSmooth(cv::Mat depthMap ,IplImage* colorImage)
{

    //cv::Mat temp = cv::Mat::zeros(320,240,CV_64F);
    int Imgwidth =depthMap.cols;
    int Imgheight = depthMap.rows;
    //int regu_size=3;
    //int coutA=0;

    for (int i=0;i< Imgwidth ;i++)
    {
        for (int j=0;j< Imgheight;j++) // horizontal
        {
            float sumDepth=0.0;
            int count_=0;
          if (depthMap.at<double> (j,i) > 0.0f)
          {
            for (int radiusy = j- RegSize; radiusy<j+ RegSize; radiusy++)
                {
                    for (int radiusx =i- RegSize ; radiusx <(i+ RegSize); radiusx++)
                    {
                        if((j- (RegSize*0.5) > 0) && (i- (RegSize*0.5)>0) && (j+ (RegSize*0.5) < Imgheight) && (i+ (RegSize*0.5)< Imgwidth))
                        {

                          if (depthMap.at<double>(radiusy,radiusx) > 0.0f)
                          {
                           sumDepth+= depthMap.at<double>(radiusy,radiusx);
                           count_++;
                          }

                    }
                }
            }

            double depth = sumDepth*(1.0/count_);

              for (int radiusy = j- RegSize ; radiusy<j+  RegSize; radiusy++)
              {
                  for (int radiusx =i- RegSize ; radiusx <(i+ RegSize); radiusx++)
                      if((j- (RegSize*0.5) > 0) && (i- (RegSize*0.5)>0) && (j+ (RegSize*0.5) < Imgheight) && (i+ (RegSize*0.5)<Imgwidth))
                      {
                        if (depthMap.at<double>(radiusy,radiusx) > 0.0f)
                            {
                              depthMap.at<double>(radiusy,radiusx)=depth;
                             }
                        }
                    }
                }
           }
    }
    std::vector<v3_t> _3DPoint_vec;
    std::vector<v3_t> _color_vec;

  for (int i=0;i< Imgwidth ;i++){
      for (int j=0;j< Imgheight;j++){
          if (depthMap.at<double>(j,i) != 0.0 )
            {
                //double depth=depthMap.at<double>(j,i);
                //cout<<depth<<endl;
                double depth= depthMap.at<double>(j,i);
                v3_t _3DPoint;
                v3_t _color;

                _3DPoint.p[0]= -1*depth*((i-(Imgwidth*0.5))/300.0f);
                _3DPoint.p[1]= -1*depth*((j-(Imgheight*0.5))/300.0f);
                _3DPoint.p[2]= -1*depth;

                _color.p[2] = CV_IMAGE_ELEM (colorImage, uchar, j , (3 * i));
                _color.p[1] = CV_IMAGE_ELEM (colorImage, uchar, j , (3 * i)+1);
                _color.p[0] = CV_IMAGE_ELEM (colorImage, uchar, j , (3 * i)+2);

                _3DPoint_vec.push_back(_3DPoint);
                _color_vec.push_back(_color);

            }
        }
    }

    DumpPointsToPly("/Users/c-hchang/Desktop/OpenCVtracking/matrix.ply", _3DPoint_vec ,(int)_3DPoint_vec.size(), _color_vec);

    //FILE *f = fopen("/Users/c-hchang/Desktop/OpenCVtracking/matrix.txt", "w");
    /* Print the ply header */
    //fprintf(f, ply_header,num_points);

    //for (int i = 0; i < width; i++)
    //{
    //    for (int j =0 ;j< height;j++) {
        /* Output the vertex */
    //        fprintf(f, "%0.0f %s",depthMap.at<double>(j,i)," ");
    //      }
    //     fprintf(f,"\n");
    //}

    //fclose(f);

    //return(depthMap);

}

IplImage* plot_Stereo_imagesf(IplImage *IGray, IplImage *IGray1, int NumPts, const vector<cv::Point> pt, const vector<cv::Point> cpt)
{
    CvPoint newmatched;
    CvPoint matched;
    int  ttw  =  IGray->width+ IGray->width;
    int  ttl  =  IGray->height;
    IplImage* Imagedisplay  = cvCreateImage(cvSize(2*IGray->width,IGray->height), IGray->depth, IGray->nChannels );
    for (int i=0; i<ttl;i++)
    {
        for (int j=0; j<ttw; j++)
        {
            if (j< IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray,i,j));
            }
            if (j>IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray1,i,j-(IGray1->width)));
            }
        }
    }

    CvPoint pt1,pt2;
    pt1.x=0;
    pt1.y=0;
    pt2.x= (IGray->width)*2;
    pt2.y= (IGray->height);

    //if(skipFrame)
    //    cvRectangle(Imagedisplay,pt1 , pt2, CV_RGB(256,0,0), 4, 8, 0 );
    //else
    //    cvRectangle(Imagedisplay,pt1 , pt2, CV_RGB(0,256,0), 4, 8, 0 );

    for (int i=0;i<NumPts; i++)
    {
        newmatched.x= (int)(cpt[i].x)+(IGray->width);
        newmatched.y= (int)(cpt[i].y);
        matched.x = (int) pt[i].x ;
        matched.y = (int) pt[i].y ;

        cvLine (Imagedisplay,
               cvPoint( matched.x, matched.y ),
               cvPoint( newmatched.x, newmatched.y ),
               CV_RGB(256,256,256)
               );


        cvCircle(Imagedisplay, newmatched, 3, CV_RGB(0,255,0),2,6,0);
        cvCircle(Imagedisplay, matched, 3, CV_RGB(0,255,0), 2,6,0);

    }  
    return(Imagedisplay);
}

float MatchingProcess(int* pixelRef, IplImage*Image2, float cpx, float cpy, int Pathsize)
{
    float SSDError=0.0f;
    float RefSummation=0;
    float TargetSummation=0;

    if (Pathsize==9)
    {
        float pixelTarget[Pathsize];
        float pixelRef_f[Pathsize];

    for(int i=0;i< Pathsize;i++)
    {
    RefSummation += pixelRef[i];
    pixelRef_f[i]= pixelRef[i];
    }

     pixelTarget[0] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx-1));
     pixelTarget[1] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx));
     pixelTarget[2] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx+1));

     pixelTarget[3] = CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx-1));
     pixelTarget[4] = CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx));
     pixelTarget[5] = CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx+1));

     pixelTarget[6] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx-1));
     pixelTarget[7] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx));
     pixelTarget[8] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx+1));

    for(int i=0;i< Pathsize;i++)
         TargetSummation+=(float)pixelTarget[i];

    float ratio =TargetSummation*(1.0/RefSummation);

    SSDError+=(pixelRef_f[0]*ratio - pixelTarget[0])*(pixelRef_f[0]*ratio- pixelTarget[0]);
    SSDError+=(pixelRef_f[1]*ratio - pixelTarget[1])*(pixelRef_f[1]*ratio- pixelTarget[1]);
    SSDError+=(pixelRef_f[2]*ratio - pixelTarget[2])*(pixelRef_f[2]*ratio- pixelTarget[2]);
    SSDError+=(pixelRef_f[3]*ratio - pixelTarget[3])*(pixelRef_f[3]*ratio- pixelTarget[3]);
    SSDError+=(pixelRef_f[4]*ratio - pixelTarget[4])*(pixelRef_f[4]*ratio- pixelTarget[4]);
    SSDError+=(pixelRef_f[5]*ratio - pixelTarget[5])*(pixelRef_f[5]*ratio- pixelTarget[5]);
    SSDError+=(pixelRef_f[6]*ratio - pixelTarget[6])*(pixelRef_f[6]*ratio- pixelTarget[6]);
    SSDError+=(pixelRef_f[7]*ratio - pixelTarget[7])*(pixelRef_f[7]*ratio- pixelTarget[7]);
    SSDError+=(pixelRef_f[8]*ratio - pixelTarget[8])*(pixelRef_f[8]*ratio- pixelTarget[8]);

    }

    if (Pathsize==25 )
    {
        float pixelTarget[Pathsize];
        float pixelRef_f[Pathsize];

        for(int i=0;i< Pathsize;i++){
            RefSummation += pixelRef[i];
            pixelRef_f[i]= pixelRef[i];
        }

        pixelTarget[0] = CV_IMAGE_ELEM (Image2, uchar, int(cpy)-2,  int(cpx)-2);
        pixelTarget[1] = CV_IMAGE_ELEM (Image2, uchar, int(cpy)-2,  int(cpx)-1);
        pixelTarget[2] = CV_IMAGE_ELEM (Image2, uchar, int(cpy)-2,  int(cpx));
        pixelTarget[3] = CV_IMAGE_ELEM (Image2, uchar, int(cpy)-2,  int(cpx)+1);
        pixelTarget[4] = CV_IMAGE_ELEM (Image2, uchar, int(cpy)-2,  int(cpx)+2);

        pixelTarget[5] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx)-2);
        pixelTarget[6] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx)-1);
        pixelTarget[7] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx));
        pixelTarget[8] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx)+1);
        pixelTarget[9] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx)+2);

        pixelTarget[10] = CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx)-2);
        pixelTarget[11] = CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx)-1);
        pixelTarget[12] = CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx));
        pixelTarget[13] = CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx)+1);
        pixelTarget[14] = CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx)+2);

        pixelTarget[15] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx)-2);
        pixelTarget[16] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx)-1);
        pixelTarget[17] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx));
        pixelTarget[18] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx)+1);
        pixelTarget[19] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx)+2);

        pixelTarget[20] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+2),  int(cpx)-2);
        pixelTarget[21] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+2),  int(cpx)-1);
        pixelTarget[22] = CV_IMAGE_ELEM (Image2, uchar, int(cpy+2),  int(cpx));
        pixelTarget[23] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-2),  int(cpx)+1);
        pixelTarget[24] = CV_IMAGE_ELEM (Image2, uchar, int(cpy-2),  int(cpx)+2);

        for(int i=0;i< Pathsize;i++)
            TargetSummation+=(float)pixelTarget[i];

        float ratio =TargetSummation*(1.0/RefSummation);

        SSDError+=(pixelRef_f[0]*ratio - pixelTarget[0])*(pixelRef_f[0]*ratio- pixelTarget[0]);
        SSDError+=(pixelRef_f[1]*ratio - pixelTarget[1])*(pixelRef_f[1]*ratio- pixelTarget[1]);
        SSDError+=(pixelRef_f[2]*ratio - pixelTarget[2])*(pixelRef_f[2]*ratio- pixelTarget[2]);
        SSDError+=(pixelRef_f[3]*ratio - pixelTarget[3])*(pixelRef_f[3]*ratio- pixelTarget[3]);
        SSDError+=(pixelRef_f[4]*ratio - pixelTarget[4])*(pixelRef_f[4]*ratio- pixelTarget[4]);
        SSDError+=(pixelRef_f[5]*ratio - pixelTarget[5])*(pixelRef_f[5]*ratio- pixelTarget[5]);
        SSDError+=(pixelRef_f[6]*ratio - pixelTarget[6])*(pixelRef_f[6]*ratio- pixelTarget[6]);
        SSDError+=(pixelRef_f[7]*ratio - pixelTarget[7])*(pixelRef_f[7]*ratio- pixelTarget[7]);
        SSDError+=(pixelRef_f[8]*ratio - pixelTarget[8])*(pixelRef_f[8]*ratio- pixelTarget[8]);
        SSDError+=(pixelRef_f[9]*ratio - pixelTarget[9])*(pixelRef_f[9]*ratio- pixelTarget[9]);
        SSDError+=(pixelRef_f[10]*ratio - pixelTarget[10])*(pixelRef_f[10]*ratio- pixelTarget[10]);
        SSDError+=(pixelRef_f[11]*ratio - pixelTarget[11])*(pixelRef_f[11]*ratio- pixelTarget[11]);
        SSDError+=(pixelRef_f[12]*ratio - pixelTarget[12])*(pixelRef_f[12]*ratio- pixelTarget[12]);
        SSDError+=(pixelRef_f[13]*ratio - pixelTarget[13])*(pixelRef_f[13]*ratio- pixelTarget[13]);
        SSDError+=(pixelRef_f[14]*ratio - pixelTarget[14])*(pixelRef_f[14]*ratio- pixelTarget[14]);
        SSDError+=(pixelRef_f[15]*ratio - pixelTarget[15])*(pixelRef_f[15]*ratio- pixelTarget[15]);
        SSDError+=(pixelRef_f[16]*ratio - pixelTarget[16])*(pixelRef_f[16]*ratio- pixelTarget[16]);
        SSDError+=(pixelRef_f[17]*ratio - pixelTarget[17])*(pixelRef_f[17]*ratio- pixelTarget[17]);
        SSDError+=(pixelRef_f[18]*ratio - pixelTarget[18])*(pixelRef_f[18]*ratio- pixelTarget[18]);
        SSDError+=(pixelRef_f[19]*ratio - pixelTarget[19])*(pixelRef_f[19]*ratio- pixelTarget[19]);
        SSDError+=(pixelRef_f[20]*ratio - pixelTarget[20])*(pixelRef_f[20]*ratio- pixelTarget[20]);
        SSDError+=(pixelRef_f[21]*ratio - pixelTarget[21])*(pixelRef_f[21]*ratio- pixelTarget[21]);
        SSDError+=(pixelRef_f[22]*ratio - pixelTarget[22])*(pixelRef_f[22]*ratio- pixelTarget[22]);
        SSDError+=(pixelRef_f[23]*ratio - pixelTarget[23])*(pixelRef_f[23]*ratio- pixelTarget[23]);
        SSDError+=(pixelRef_f[24]*ratio - pixelTarget[24])*(pixelRef_f[24]*ratio- pixelTarget[24]);

    }

    return(SSDError);
}
void ReferencePatch(IplImage* Image1, int PathSize, int* pixelRef, const int x, const int y){

    int Size_=PathSize;

    if (PathSize==9)
    {
    int ptemp[PathSize];

    ptemp[0] = CV_IMAGE_ELEM (Image1, uchar, y-1,  x-1);
    ptemp[1] = CV_IMAGE_ELEM (Image1, uchar, y-1,  x);
    ptemp[2] = CV_IMAGE_ELEM (Image1, uchar, y-1,  x+1);

    ptemp[3] = CV_IMAGE_ELEM (Image1, uchar, y,  x-1);
    ptemp[4] = CV_IMAGE_ELEM (Image1, uchar, y,  x);
    ptemp[5] = CV_IMAGE_ELEM (Image1, uchar, y,  x+1);

    ptemp[6] = CV_IMAGE_ELEM (Image1, uchar, y+1,  x-1);
    ptemp[7] = CV_IMAGE_ELEM (Image1, uchar, y+1,  x);
    ptemp[8] = CV_IMAGE_ELEM (Image1, uchar, y+1,  x+1);
      memcpy(pixelRef,ptemp,sizeof(int)*Size_);
    }
    if (PathSize==25)
    {
    int ptemp[Size_];

    ptemp[0] = CV_IMAGE_ELEM (Image1, uchar, y-2,  x-2);
    ptemp[1] = CV_IMAGE_ELEM (Image1, uchar, y-2,  x-1);
    ptemp[2] = CV_IMAGE_ELEM (Image1, uchar, y-2,  x);
    ptemp[3] = CV_IMAGE_ELEM (Image1, uchar, y-2,  x+1);
    ptemp[4] = CV_IMAGE_ELEM (Image1, uchar, y-2,  x+2);

    ptemp[5] = CV_IMAGE_ELEM (Image1, uchar, y-1,  x-2);
    ptemp[6] = CV_IMAGE_ELEM (Image1, uchar, y-1,  x-1);
    ptemp[7] = CV_IMAGE_ELEM (Image1, uchar, y-1,  x);
    ptemp[8] = CV_IMAGE_ELEM (Image1, uchar, y-1,  x+1);
    ptemp[9] = CV_IMAGE_ELEM (Image1, uchar, y-1,  x+2);

    ptemp[10] = CV_IMAGE_ELEM (Image1, uchar, y,  x-2);
    ptemp[11] = CV_IMAGE_ELEM (Image1, uchar, y,  x-1);
    ptemp[12] = CV_IMAGE_ELEM (Image1, uchar, y,  x);
    ptemp[13] = CV_IMAGE_ELEM (Image1, uchar, y,  x+1);
    ptemp[14] = CV_IMAGE_ELEM (Image1, uchar, y,  x+2);

    ptemp[15] = CV_IMAGE_ELEM (Image1, uchar, y+1,  x-2);
    ptemp[16] = CV_IMAGE_ELEM (Image1, uchar, y+1,  x-1);
    ptemp[17] = CV_IMAGE_ELEM (Image1, uchar, y+1,  x);
    ptemp[18] = CV_IMAGE_ELEM (Image1, uchar, y+1,  x+1);
    ptemp[19] = CV_IMAGE_ELEM (Image1, uchar, y+1,  x+2);

    ptemp[20] = CV_IMAGE_ELEM (Image1, uchar, y+2,  x-2);
    ptemp[21] = CV_IMAGE_ELEM (Image1, uchar, y+2,  x-1);
    ptemp[22] = CV_IMAGE_ELEM (Image1, uchar, y+2,  x);
    ptemp[23] = CV_IMAGE_ELEM (Image1, uchar, y+2,  x+1);
    ptemp[24] = CV_IMAGE_ELEM (Image1, uchar, y+2,  x+2);
           memcpy(pixelRef,ptemp,sizeof(int)*Size_);
    }
}
void EpipolarMatching_1 (double* R_relative, double* T_relative, double* Kmatrix , double* Tmatrix,  IplImage* Image1, IplImage* Image2, IplImage* colorImage1, IplImage* colorImage2, int frameIndex)
{
    int ImgWidth = Image1->width;
    int ImgHeight = Image1->height;

    int ImgCenterWidth=  (ImgWidth*0.5);
    int ImgCenterHeight= (ImgHeight*0.5);

    cv::Mat DepthMap = cv::Mat::zeros(ImgHeight,ImgWidth,CV_64F);

    //double Kmatrix[9], Rmatrix[9], Tmatrix[3];
    //input.PopKMattix(1, Kmatrix);
    //input.PopTcMatrix(1, Tmatrix);
    //input.PopRotcMatrix(1, Rmatrix);

    Eigen::Vector3d TMat;  // the host matrix for x
    memcpy(TMat.data(), T_relative, sizeof(double)*3*1);
    TMat[2]=-1*TMat[2];


    //Eigen::Vector3d TRelative;
    //memcpy(TRelative.data(), Trelative , sizeof(double)*3*1);


    Eigen::MatrixXd RotT(3, 3); // the host matrix for X
    memcpy(RotT.data(), R_relative, sizeof(double)*9);
    Eigen::MatrixXd RMat = RotT.transpose();


    //Eigen::MatrixXd RRelativeT(3, 3); // the host matrix for X
    //memcpy(RRelativeT.data(), Rrelative, sizeof(double)*9);
    //Eigen::MatrixXd RRelativeMat = RRelativeT.transpose();


    Eigen::MatrixXd KMat(3, 3); // the host matrix for X
    memcpy(KMat.data(), Kmatrix, sizeof(double)*9);
    KMat(0,2)= 1*ImgWidth*0.5f;
    KMat(1,2)= 1*ImgHeight*0.5f;

    //float pixelx , pixely, pixelDy , pixelDx;

    IplImage* imgB;
    imgB  = cvCloneImage(Image1);
    IplImage* imgC;
    imgC  = cvCloneImage(Image2);

    vector<cv::Point> pt;
    vector<cv::Point> cpt;

    for(int y= MIN_HEIGHT ;y< ImgHeight-MAX_HEIGHT; y++)

        for(int x = MIN_WIDTH ;x<ImgWidth- MAX_WIDTH ;x++){

            // search depth and finding corresponding points  //
            float  MaxIdepth= 0.001f;    // set MaxDepth 100;
            float  MinIdepth= 0.5f;     // set MinDepth 5;
            float  incx=0.0f;
            float  incy=0.0f;
            int    cpxfinal=0;
            int    cpyfinal=0;
            float  MinError =  99999999.0f;

            Eigen::Vector3d pInf;

            bool goodPoint = Match_Main (Image1, Image2 ,  x ,  y , pInf , MaxIdepth,  MinIdepth , KMat, RMat, TMat,  cpxfinal,  cpyfinal ,  incx , incy, MinError);

            if (goodPoint) {

                //if(test==1001){
                //    cvCircle(imgC,cv::Point(cpxfinal,cpyfinal), 2.0 , CV_RGB(255,255,255),3.0,8,0);
                //    cvCircle(imgB,cv::Point(x,y), 0.5 , CV_RGB(255,255,255),3.0,8,0);
                //   cvShowImage("new1", imgC);
                //   cvShowImage("new", imgB);
                //   cvWaitKey();
                //}
                //test++;

                if ( MinError < MAX_ERROR)
                {

                    if(incx*incx>incy*incy)
                        DepthRecovery(ImgCenterWidth, ImgCenterHeight, cpxfinal, cpyfinal , TMat /*absolute pose*/, DepthMap, pInf);

                    else {
                        Eigen::Vector3d KPnew =  Eigen::Vector3d(cpxfinal,cpyfinal,1.0f);
                        Eigen::Vector3f Xnew= KPnew.cast<float>()-pInf.cast<float>();
                        Xnew = Xnew* (1.0/pInf[2]);
                        double x_3d = (double(cpxfinal)-ImgCenterWidth);
                        double y_3d = (double(cpyfinal)-ImgCenterHeight);
                        Eigen::Vector3f Tk;
                        Tk[0]= 300.0f*TMat[0];
                        Tk[1]= 300.0f*TMat[1];
                        float depth = -1*Tk[2]/Xnew[2];
                        cout << -1*depth*(x_3d/300.0f)<<" "<<-1*depth*(y_3d/300.0f)<<" "<<depth<<endl;
                    }
                }
            }
        }

    DepthSmooth_1(DepthMap, colorImage2, Tmatrix , frameIndex);

    /*
     FILE *file_ = fopen("/Users/c-hchang/Desktop/OpenCVtracking/matrix.txt", "w");
     for (int i = 0; i < ImgWidth ; i++)
     {
     for (int j =0 ;j< ImgHeight ;j++) {
     // Output the vertex //
     fprintf(file_, "%0.0f %s", DepthMap.at<double>(j,i)," ");
     }
     fprintf(file_,"\n");
     }
     
     fclose(file_);
     */
    
}
void  DepthSmooth_1 (cv::Mat depthMap ,IplImage* colorImage, double* Tmatrix, int frameIndex)
{

    //cv::Mat temp = cv::Mat::zeros(320,240,CV_64F);
    int Imgwidth =depthMap.cols;
    int Imgheight = depthMap.rows;
    //int regu_size=3;
    //int coutA=0;

    for (int i=0;i< Imgwidth ;i++)
    {
        for (int j=0;j< Imgheight;j++) // horizontal
        {
            float sumDepth=0.0;
            int count_=0;
            if (depthMap.at<double> (j,i) > 0.0f)
            {
                for (int radiusy = j- RegSize; radiusy<j+ RegSize; radiusy++)
                {
                    for (int radiusx =i- RegSize ; radiusx <(i+ RegSize); radiusx++)
                    {
                        if((j- (RegSize*0.5) > 0) && (i- (RegSize*0.5)>0) && (j+ (RegSize*0.5) < Imgheight) && (i+ (RegSize*0.5)< Imgwidth))
                        {

                            if (depthMap.at<double>(radiusy,radiusx) > 0.0f)
                            {
                                sumDepth+= depthMap.at<double>(radiusy,radiusx);
                                count_++;
                            }

                        }
                    }
                }

                double depth = sumDepth*(1.0/count_);

                for (int radiusy = j- RegSize ; radiusy<j+  RegSize; radiusy++)
                {
                    for (int radiusx =i- RegSize ; radiusx <(i+ RegSize); radiusx++)
                        if((j- (RegSize*0.5) > 0) && (i- (RegSize*0.5)>0) && (j+ (RegSize*0.5) < Imgheight) && (i+ (RegSize*0.5)<Imgwidth))
                        {
                            if (depthMap.at<double>(radiusy,radiusx) > 0.0f)
                            {
                                depthMap.at<double>(radiusy,radiusx)=depth;
                            }
                        }
                }
            }
        }
    }


    ExportDepthData(colorImage,depthMap, frameIndex, Tmatrix);

}
void ExportDepthData(IplImage* colorImage , cv::Mat depthMap, int frameIndex, double*Tmatrix ){

    int Imgwidth =depthMap.cols;
    int Imgheight = depthMap.rows;

    std::vector<v3_t> _3DPoint_vec;
    std::vector<v3_t> _color_vec;

    for (int i=0;i< Imgwidth ;i++){
        for (int j=0;j< Imgheight;j++){
            if (depthMap.at<double>(j,i) != 0.0 )
            {
                //double depth=depthMap.at<double>(j,i);
                //cout<<depth<<endl;
                double depth= depthMap.at<double>(j,i);
                v3_t _3DPoint;
                v3_t _color;

                _3DPoint.p[0]= -1*depth*((i-(Imgwidth*0.5))/300.0f)+Tmatrix[0];
                _3DPoint.p[1]= -1*depth*((j-(Imgheight*0.5))/300.0f)+Tmatrix[1];
                _3DPoint.p[2]= -1*depth+Tmatrix[2];

                _color.p[2] = CV_IMAGE_ELEM (colorImage, uchar, j , (3 * i));
                _color.p[1] = CV_IMAGE_ELEM (colorImage, uchar, j , (3 * i)+1);
                _color.p[0] = CV_IMAGE_ELEM (colorImage, uchar, j , (3 * i)+2);

                _3DPoint_vec.push_back(_3DPoint);
                _color_vec.push_back(_color);

            }
        }
    }
    std::string s = std::to_string(frameIndex);

    string message ="/Users/c-hchang/Desktop/OpenCVtracking/matrix"+s+".ply";
    cout<< "message: " << message<<endl;

    char *y = new char[message.length() + 1]; // or
    std::strcpy(y, message.c_str());

    // char y[100];
    DumpPointsToPly( y, _3DPoint_vec ,(int)_3DPoint_vec.size(), _color_vec);

    delete [] y;

    //FILE *f = fopen("/Users/c-hchang/Desktop/OpenCVtracking/matrix.txt", "w");
    /* Print the ply header */
    //fprintf(f, ply_header,num_points);

    //for (int i = 0; i < width; i++)
    //{
    //    for (int j =0 ;j< height;j++) {
    /* Output the vertex */
    //        fprintf(f, "%0.0f %s",depthMap.at<double>(j,i)," ");
    //      }
    //     fprintf(f,"\n");
    //}
    //fclose(f);
    
    //return(depthMap);
}


DMap::DMap( int ImgWidth, int ImgHeight){

    Initial_DepthData(ImgWidth, ImgWidth);
}

void DMap::Initial_DepthData(int ImgWidth, int ImgHeight){

    DepthData Dp;

    Dp.depthMap = cv::Mat::zeros(ImgHeight,ImgWidth,CV_64F);

    Dp.Dxy = cv::Mat::zeros(ImgHeight,ImgWidth,CV_16U);

    DMapVector.push_back(Dp);
}

//bool DMap::checkAvailable(int x, int y){
//
//     DMapVector.end().vec
//
//}
