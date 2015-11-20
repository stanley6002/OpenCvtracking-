//
//  Map3DStruct.cpp
//  OpenCVtracking
//
//  Created by C-HChang on 11/19/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#include "Map3DStruct.h"
#include <math.h>
using namespace std;

void _3DPt::Initial(int NumPts, vector<v3_t> pt, vector<v2_t> leftLocation, vector<v2_t> RightLocation, int FrameNumber)
{
    _3D.reserve(NumPts);

    for (int i=0;i< NumPts;i++){
        Initialization(pt[i], leftLocation[i], RightLocation[i], FrameNumber);
    }
}
_3DPt::_3DPt(){

}

void _3DPt::Initialization(v3_t pt, v2_t LeftLocation, v2_t RightLocation, int LatestFrame)
{
    Pt3D temp3D;
    temp3D.Point[0]=pt.p[0];
    temp3D.Point[1]=pt.p[1];
    temp3D.Point[2]=pt.p[2];

    _2DFea temp2DL;
    _2DFea temp2DR;

    temp2DL._2DPt= LeftLocation;
    temp2DL.FrameNum= LatestFrame-1;
    temp2DR._2DPt= RightLocation;
    temp2DR.FrameNum= LatestFrame;
    temp3D._2D.push_back(temp2DL);
    temp3D._2D.push_back(temp2DR);

    _3D.push_back(temp3D);
}
void _3DPt:: ConnectNewFrame(int NumPts ,v2_t* leftLocation /* current left point*/ , v2_t* RightLocation /*current new frame points*/ , int FrameNumber, vector<v2_t>&Reproject2D, vector<v3_t>& Project3D)
 {

    //int Previous_ptsize   =   (int)Previous_pts.size();
    int ConnectedPtsize   = NumPts;

    // serach frame list in here //
    int LowboundSearch;
    int UpperboundSearch;
    int CurrentListIndex=_3DPt::Numof3Dpts();

     if(CurrentListIndex >= 400) {
        LowboundSearch= CurrentListIndex-400;
        UpperboundSearch= CurrentListIndex;
    }

    else{
        LowboundSearch= 0;
        UpperboundSearch= CurrentListIndex;
    }

    int Memsize = UpperboundSearch - LowboundSearch;

    int* tempCurrent  = new int [ConnectedPtsize];

     for (int i=0;i< ConnectedPtsize;i++)
         tempCurrent[i]= VALID;

    int numreprojection =0;

    for (int i=0; i< Memsize; i++)
    {
        int shiftlowbound = i + LowboundSearch;
         //int FrameRowsize = (int) FM_v2_frame[i].size();
        if (LatestCamera(shiftlowbound) == FrameNumber-1)
        {
           if (ReadReproFlag(shiftlowbound) == 0)  // check this flag has been activated or not //
              {
                int x = _3D[shiftlowbound]._2D.back()._2DPt.p[0];
                int y = _3D[shiftlowbound]._2D.back()._2DPt.p[1];

                 for (int j=0; j< ConnectedPtsize; j++)
                    {
                     if (tempCurrent[j] == VALID)
                        {
                            int x_m =  (int) leftLocation[j].p[0];
                            int y_m =  (int) leftLocation[j].p[1];

                         if (sqrt(((x-x_m)*(x-x_m))+((y-y_m)*(y-y_m)))<=1.414)
                           {
                            numreprojection++;
                            ActivateReproFlag(shiftlowbound);
                            ActivateReproIndx(shiftlowbound,j);
                            tempCurrent[j]=0;
                            break;
                          }
                     }
                 }
             }
         }
     }

     Reproject2D.reserve(numreprojection);
     Project3D.reserve(numreprojection);


     for (int i=0; i< Memsize; i++)
     {
         int shiftlowbound = i + LowboundSearch;
         if (ReadReproFlag(shiftlowbound))
         {
               Project3D.push_back(Read3DPoint(shiftlowbound));
               Reproject2D.push_back(RightLocation[ReadReproIndx(shiftlowbound)]);
         }
     }
 }

void _3DPt:: _3DPtGeneration( int NumPts , int FrameNum,  v2_t* leftLocation, v2_t* RightLocation,  vector<vector<v2_t> >&V2Location, vector<vector<int> >&V2Frame)
{

    int LowboundSearch, UpperboundSearch;
    int CurrentListIndex=_3DPt::Numof3Dpts();

    //vector<vector<v2_t> > V2Location /*2D points location*/ ;
    //vector<vector<int> >  V2Frame    /*frame number*/;
    //vector<v3_t> v3Pts ;


    int *Overlap = new int [NumPts];
    bool *RemovEIdx = new bool [NumPts];

    memset(RemovEIdx,0,NumPts*sizeof(int));
    memset(Overlap,0, NumPts*sizeof(int));



    if(CurrentListIndex >= 400) {
         LowboundSearch= CurrentListIndex-400;
         UpperboundSearch= CurrentListIndex;
    }
    else{
        LowboundSearch= 0;
        UpperboundSearch= CurrentListIndex;
    }

    // Search Overlapped point //
    int Memsize = UpperboundSearch - LowboundSearch;

    for (int i=0; i< Memsize ; i++)
    {
        int shiftlowbound = i + LowboundSearch;
        if (ReadReproFlag(shiftlowbound))
        {
            /*ReproInx ->2D index*/
            int ReproInx = ReadReproIndx(shiftlowbound);
            Overlap[ReproInx]= shiftlowbound;

            _2DFea new_pt;
            new_pt.FrameNum= FrameNum;
            new_pt._2DPt = RightLocation[ReproInx];

            vector<_2DFea> temp =  Read2DPt(shiftlowbound);
            temp.push_back(new_pt);

            AddtoMap(temp, V2Location, V2Frame);
        }
    }

    // ADD new points for triangulation//
    for (int i=0;i< NumPts ;i++)
    {
      if (Overlap[i]==0)
      {
          vector<_2DFea> tempvec;
          _2DFea tempL,  tempR;
          
          tempL._2DPt = leftLocation[i];
          tempL.FrameNum = FrameNum-1;

          tempR._2DPt = RightLocation[i];
          tempR.FrameNum = FrameNum;

          tempvec.push_back(tempL);
          tempvec.push_back(tempR);

          AddtoMap(tempvec, V2Location, V2Frame);
      }
    }

    //for (int i=0;i<NumPts;i++){
    //    cout<<Overlap[i]<<endl;
    //}

    //vector<bool> boolvector;
    //ScameraPose.Triangulation_N_frame_Map(0, 0  , 0 , V2Location, V2Frame, v3Pts, boolvector);


}