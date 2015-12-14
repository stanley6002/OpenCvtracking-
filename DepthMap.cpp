//
//  DepthMap.cpp
//  OpenCVtracking
//
//  Created by C-HChang on 12/10/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#include "DepthMap.h"
#include "eigen3/Eigen/Dense"


#define MIN_WIDTH  30
#define MAX_WIDTH  30
#define MIN_HEIGHT 30
#define MAX_HEIGHT 30
#define MAXGRA 8000
# define SSDWindow 9

// match first two initial frame//
void EpipolarMatching(CameraPose input, IplImage* Image1, IplImage* Image2)
{
    int ImgWidth = Image1->width;
    int ImgHeight = Image1->height;

    //cv::Mat m11(ImgHeight, ImgWidth, CV_8UC1); //3-channel
    cv::Mat f = cv::Mat::zeros(ImgHeight,ImgWidth,CV_64F);

    //imshow("f",f);

    float Depth[ImgWidth*ImgHeight];

    double Kmatrix[9], Rmatrix[9], Tmatrix[3];

    input.PopKMattix(1, Kmatrix);
    input.PopTcMatrix(1, Tmatrix);
    input.PopRotcMatrix(1, Rmatrix);


    Eigen::Vector3d TMat;
    memcpy(TMat.data(), Tmatrix, sizeof(double)*3*1);
    TMat[2]=-1*TMat[2];

    Eigen::MatrixXd RotT(3, 3); // the host matrix for X
    memcpy(RotT.data(), Rmatrix, sizeof(double)*9);
    Eigen::MatrixXd RMat = RotT.transpose();

    Eigen::MatrixXd KMat(3, 3); // the host matrix for X
    memcpy(KMat.data(), Kmatrix, sizeof(double)*9);
    KMat(0,2)= 1*ImgWidth*0.5f;
    KMat(1,2)= 1*ImgHeight*0.5f;


    float pixelx , pixely, pixelDy , pixelDx;

    IplImage* imgB;
    imgB  = cvCloneImage(Image1);
    IplImage* imgC;
    imgC  = cvCloneImage(Image2);
    int Plot=0;

    vector<cv::Point> pt;
    vector<cv::Point> cpt;

    int test=0;

    for(int y= MIN_HEIGHT ;y< ImgHeight-MAX_HEIGHT; y++)

        for(int x = MIN_WIDTH ;x<ImgWidth- MAX_WIDTH ;x++){


        pixelx= CV_IMAGE_ELEM (Image1, uchar, y,  x+1) - CV_IMAGE_ELEM (Image1, uchar, y,  x-1);
        pixely= CV_IMAGE_ELEM (Image1, uchar, y-1,  x) - CV_IMAGE_ELEM (Image1, uchar, y+1,  x);
        pixelDy= CV_IMAGE_ELEM (Image1, uchar, y-1,  x-1) - CV_IMAGE_ELEM (Image1, uchar, y+1,  x+1);
        pixelDx= CV_IMAGE_ELEM (Image1, uchar, y+1,  x-1) - CV_IMAGE_ELEM (Image1, uchar, y-1,  x+1);


        float eplGradSquared = pixelx * pixelx + pixely * pixely+ pixelDy*pixelDy;

        // search depth and finding corresponding points  //
        float  MaxIdepth= 0.001f;    // set MaxDepth 100;
        float  MinIdepth= 0.5f;     // set MinDepth 5;

        //float  MinIdepth= 1.0f;

        if (eplGradSquared> MAXGRA) {
             
                //Eigen::Vector3d KinvP =  Eigen::Vector3d(x,y,1.0f);
             Eigen::Vector3d KinvP =  Eigen::Vector3d(x,y,1.0f);
             Eigen::Vector3d pInf =   KMat*RMat*KMat.inverse()*KinvP;

             Eigen::Vector3f ClosePt = pInf.cast<float>() + KMat.cast<float>() *TMat.cast<float>() * MinIdepth;
             Eigen::Vector3f FarPt =  pInf.cast<float>() + KMat.cast<float>()  *TMat.cast<float>() * MaxIdepth;

             ClosePt = ClosePt / ClosePt[2];
             FarPt = FarPt/FarPt[2];

                //cout<< ClosePt<<endl;
                //cout<<FarPt<<endl;

             float incx = ClosePt[0] - FarPt[0];
             float incy = ClosePt[1] - FarPt[1];

             float eplLength = sqrt(incx*incx+incy*incy);

             incx *= 1.0f/eplLength;
             incy *= 1.0f/eplLength;

             float cpx =  FarPt[0];
             float cpy =  FarPt[1];

             int  counter=0;
             int pixelRef[25];

                ReferencePatch(Image1, 25 /*size*/ ,  pixelRef, x, y);

                //float SSDError;
                int cpxfinal=0;
                int cpyfinal=0;
                float MixError =  99999999.0;


                while (((cpx) >1 && (cpy)>1 && (cpx) <ImgWidth-1 && (cpy)<ImgHeight-1) && counter< 13)  // need new boundry test//
                 {

                   float SSDError=0;

                   int pixelx= CV_IMAGE_ELEM (Image2, uchar, int(cpy),   int(cpx+1)) - CV_IMAGE_ELEM (Image2, uchar, int(cpy),  int(cpx-1));
                   int pixely= CV_IMAGE_ELEM (Image2, uchar, int(cpy-1), int(cpx)) - CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx));
                   int pixelDy= CV_IMAGE_ELEM (Image2, uchar,int(cpy-1),  int(cpx-1)) - CV_IMAGE_ELEM (Image2, uchar, int(cpy+1),  int(cpx+1));
                   int pixelDx= CV_IMAGE_ELEM (Image2, uchar,int(cpy+1),  int(cpx-1)) - CV_IMAGE_ELEM (Image2, uchar, int(cpy-1),  int(cpx+1));

                   float eplGradSquared = pixelx * pixelx + pixely * pixely+ pixelDy*pixelDy+  pixelDx*pixelDx;

                   if(eplGradSquared> 5000){

                        SSDError = MatchingProcess(pixelRef, Image2,  cpx,  cpy, 25);

                       if(SSDError<MixError)
                        {
                         cpxfinal = cpx;
                         cpyfinal = cpy;
                         MixError = SSDError;
                       }
                    }

                     cpx+= incx;
                     cpy+= incy;
                     counter++;
                 }

                 //if(test==1001){
                 //    cvCircle(imgC,cv::Point(cpxfinal,cpyfinal), 2.0 , CV_RGB(255,255,255),3.0,8,0);
                 //    cvCircle(imgB,cv::Point(x,y), 0.5 , CV_RGB(255,255,255),3.0,8,0);
                 //   cvShowImage("new1", imgC);
                 //   cvShowImage("new", imgB);
                 //   cvWaitKey();
                //}

                test++;


                if (MixError<50000.0f){

                    if(incx*incx>incy*incy)
                    {
                        float oldX = cpxfinal ;
                        float nominator = 300*(TMat[0]-TMat[2]);

                        Eigen::Vector3d KPnew =  Eigen::Vector3d(cpxfinal,cpyfinal,1.0f);

                         //cvCircle(imgB,cv::Point(x,y), 0.5 , CV_RGB(255,255,255),3.0,8,0);
                        //cvCircle(imgC,cv::Point(cpxfinal,cpyfinal), 0.5 , CV_RGB(255,255,255),3.0,8,0);

                        KPnew[0]=KPnew[0]-320;
                        KPnew[1]=KPnew[1]-240;

                        pInf[0]=pInf[0]-320;
                        pInf[1]=pInf[1]-240;

                        Eigen::Vector3f Xnew= KPnew.cast<float>()-pInf.cast<float>();

                        Eigen::Vector3f Tk;
                        Tk[0]=  300.0f*TMat[0];
                        Tk[1]=  300.0f*TMat[1];

                        float depth = (-1*Tk[0]/Xnew[0])*(1.0/pInf[2]);

                        double x_3d = (double(cpxfinal)-320);
                        double y_3d = (double(cpyfinal)-240);
                        double x_ref = (double(x)-320);
                        double y_ref = (double(y)-240);

                        if(abs(depth)<40 && depth<0){
                        // cvCircle(imgC,cv::Point( cpxfinal, cpyfinal), 0.5 , CV_RGB(255,255,255),3.0,8,0);
                        // cvCircle(imgB,cv::Point( x, y), 0.5 , CV_RGB(255,255,255),3.0,8,0);

                            cout<<"before : "<< -1*depth*(x_3d/300.0f)<<" "<<-1*depth*(y_3d/300.0f)<<" "<<depth<<endl;
                            f.at<double>(y_3d+240, x_3d+320)=fabs(depth);
                        }

                         /* triangulation
                        double R1matrix[9]={1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};
                        double t1matrix[3]={0.0,0.0,0.0};
                        double R_relative[9]={RMat(0,0),RMat(0,1),RMat(0,2),RMat(1,0),RMat(1,1),RMat(1,2),RMat(2,0),RMat(2,1),RMat(2,2)};
                        double t_relative[3]={TMat[0],TMat[1],TMat[2]};
                        double K1matrix[9]={300.0,0.0,0.0,0.0,300.0,0.0,0.0,0.0,1.0};
                        double K2matrix[9]={300.0,0.0,0.0,0.0,300.0,0.0,0.0,0.0,1.0};

                        bool in_front = true;
                        double angle = 0.0;
                        double error_tr;
                        v3_t temp;
                        v2_t p;
                        v2_t q;
                        p.p[0] = x_3d;
                        p.p[1] = y_3d;
                        q.p[0] = x_ref;
                        q.p[1] = y_ref;

                        temp = Triangulate(p, q, R1matrix , t1matrix ,R_relative, t_relative, error_tr, in_front, angle ,true,K1matrix,K2matrix);
                        if(temp.p[2]<0 && abs(temp.p[2])<40)
                            cout<< temp.p[0]<<" "<<temp.p[1]<<" "<<temp.p[2]<<" "<<x_3d-x_ref<<" "<< x_3d +320 <<endl;
                        */
                        
                      }
                    else {
                        Eigen::Vector3d KPnew =  Eigen::Vector3d(cpxfinal,cpyfinal,1.0f);
                        Eigen::Vector3f Xnew= KPnew.cast<float>()-pInf.cast<float>();
                        Xnew = Xnew* (1.0/pInf[2]);
                        double x_3d = (double(cpxfinal)-320);
                        double y_3d = (double(cpyfinal)-240);
                       Eigen::Vector3f Tk;
                       Tk[0]= 300.0f*TMat[0];
                       Tk[1]= 300.0f*TMat[1];
                       float depth = -1*Tk[2]/Xnew[2];
                       cout << -1*depth*(x_3d/300.0f)<<" "<<-1*depth*(y_3d/300.0f)<<" "<<depth<<endl;
                    }
               }

            }
     }

    DepthSmooth(f);
    //IplImage* stereoImag = plot_Stereo_imagesf(imgB, imgC, 3000 , pt, cpt);
    //cvShowImage("new", stereoImag);
    imshow("f",f);
    cvShowImage("new", imgB);
    cvShowImage("new1", imgC);
    cvWaitKey('p');
    //cvWaitKey('p');
}

cv::Mat DepthSmooth(cv::Mat depthMap){

    cv::Mat temp = cv::Mat::zeros(320,240,CV_64F);
    int width =depthMap.cols;
    int height = depthMap.rows;
    int regu_size=3;

    for (int i=0;i< width ;i++)
    {
        for (int j=0;j< height;j++)
        {
            float sumDepth=0.0;
            int count_=0;
          if (depthMap.at<double>(j,i)!= 0.0)
          {
            for (int radiusy = j - 2; radiusy< (j+1); radiusy++)
                {
                    for (int radiusx = i- 4; radiusx < (i+3); radiusx++)
                    {
                        if((j-regu_size>0) && (i-regu_size>0) && (j+ regu_size<height) && (i+regu_size<width))
                        {

                          if (depthMap.at<double>(radiusy,radiusx)!=0.0)
                          {
                          sumDepth+= depthMap.at<double>(radiusy,radiusx);
                          count_++;
                          }

                    }
                }
            }
            //if (count_ != 0)
            //{
                double depth = sumDepth*(1.0/count_);
                //cout<<depth<<endl;
                //cout<<sumDepth*(1.0/count_)<<" "<<count_<<endl;
                cout << -1*depth*((i-320)/300.0f)<<" "<<-1*depth*((j-240)/300.0f)<<" "<<-1*depth<<endl;
                depthMap.at<double>(j,i)= depth;
          }   //depthMap.at<double>(j,i)= sumDepth*(1.0/count_);
        }
     }

//  for (int i=0;i< width ;i++){
//      for (int j=0;j< height;j++){
//          if (depthMap.at<double>(j,i)> 5.0 )
//            {
//                double depth=depthMap.at<double>(j,i);
//                //cout<<depth<<endl;
//                //depth= depthMap.at<double>(j,i);
//                cout << -1*depth*((i-320)/300.0f)<<" "<<-1*depth*((j-240)/300.0f)<<" "<<-1*depth<<endl;
//           // depthMap.at<double>(j,i)= depth;
//
//            }
//        }
//    }

     return(temp);
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

        cvLine(Imagedisplay,
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

    for(int i=0;i< Pathsize;i++){
          RefSummation += pixelRef[i];
        pixelRef_f[i]= pixelRef[i]; }

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
            pixelRef_f[i]= pixelRef[i]; }

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
