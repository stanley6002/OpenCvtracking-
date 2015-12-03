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
    temp3D.NewPt = true;

    _3D.push_back(temp3D);
}
void _3DPt:: ConnectNewFrame(int NumPts ,v2_t* leftLocation /* current left point*/ , v2_t* RightLocation /*current new frame points*/ , int FrameNumber, vector<v2_t>&Reproject2D, vector<v3_t>& Project3D)
 {

    //int Previous_ptsize   =   (int)Previous_pts.size();
    int ConnectedPtsize   = NumPts;

    // serach frame list in here //
    int LowboundSearch;
    int UpperboundSearch;
    int CurrentListIndex= _3DPt::Numof3Dpts();

     if(CurrentListIndex >= 600) {
        LowboundSearch= CurrentListIndex-600;
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

    // check newpt flag //
    // check newpt flag //
    // for (int i=0;i< ConnectedPtsize;i++){
    //     cout<<"Right location "<<RightLocation[i].p[0]<<" "<<RightLocation[i].p[1]<<endl;
    // }

    int numreprojection =0;

    for (int i=0; i< Memsize; i++)
    {

        int shiftlowbound = i + LowboundSearch;
        //int FrameRowsize = (int) FM_v2_frame[i].size();
        if (LatestCamera(shiftlowbound) == FrameNumber-1)
          {
           if (ReadReproFlag(shiftlowbound) == 0 && _3D[shiftlowbound].NewPt ==1)
         //if (ReadReproFlag(shiftlowbound) == 0)
             // check this flag has been activated or not //

              {
                int x = _3D[shiftlowbound]._2D.back()._2DPt.p[0];
                int y = _3D[shiftlowbound]._2D.back()._2DPt.p[1];
                 // cout<<x<<" "<<y<<endl;


                 for (int j=0; j< ConnectedPtsize; j++)
                    {
                     if (tempCurrent[j] == VALID)
                        {
                            int x_m =  (int) leftLocation[j].p[0];
                            int y_m =  (int) leftLocation[j].p[1];

                            //int x_t =  (int) RightLocation[j].p[0];
                            //int y_t =  (int) RightLocation[j].p[1];

                         if (sqrt(((x-x_m)*(x-x_m))+((y-y_m)*(y-y_m)))<=1.414)
                           {
                            //cout<<x_t<<" "<<y_t<<endl;
                            numreprojection++;

                            //cout<<_3D[i].Point[0]<<" "<< _3D[i].Point[1]<<" "<< _3D[i].Point[2]<<endl;
                            ActivateReproFlag(shiftlowbound);

                            // set the 3D points with corresponding 2D location //
                            ActivateReproIndx(shiftlowbound,j);
                            //cout<<"save location: "<<shiftlowbound<<" "<<j<<endl;
                            tempCurrent[j]=0;
                            break;
                          }
                     }
                 }
             }
         }
     }
     
     cout<<"Number of connected Points : "<<numreprojection<<endl;

     Reproject2D.reserve(numreprojection);
     Project3D.reserve(numreprojection);

     // collect 3D and 2D points for finding the camera pose //
     for (int i=0; i< Memsize; i++)
     {
         int shiftlowbound = i + LowboundSearch;
         if (ReadReproFlag(shiftlowbound))
         {
            Project3D.push_back(Read3DPoint(shiftlowbound));
            Reproject2D.push_back(RightLocation[ReadReproIndx(shiftlowbound)]);

         }
     }
     for (int i=0; i< Reproject2D.size(); i++)
     {
     //cout<<Reproject2D[i].p[0]<<" "<<Reproject2D[i].p[1]<<endl;
     //cout<<Project3D[i].p[0]<<" "<< Project3D[i].p[1]<<" "<<Project3D[i].p[2]<<endl;
     }
    // clean up all new point flag //
    for (int i=0; i< Memsize; i++)
    {
        int shiftlowbound = i + LowboundSearch;
          if (_3D[shiftlowbound].NewPt ==1){
             // check this flag has been activated or not //
               RemoveNewFlag(i);
        } 
     }
 

 }

void _3DPt:: _3DPtGeneration( int NumPts , int FrameNum,  v2_t* leftLocation, v2_t* RightLocation,  vector<vector<v2_t> >&V2Location, vector<vector<int> >&V2Frame, vector<int>& Overlap)
{
    // add FrameNum to current frame //
    int LowboundSearch, UpperboundSearch;
    int CurrentListIndex=_3DPt::Numof3Dpts();

    //vector<vector<v2_t> > V2Location /*2D points location*/ ;
    //vector<vector<int> >  V2Frame    /*frame number*/;
    //vector<v3_t> v3Pts ;

    for (int i=0;i< NumPts;i++){
        Overlap.push_back(-999);
        V2Location.push_back(vector<v2_t>());
        V2Frame.push_back(vector<int>());
    }
   
    //int *Overlap = new int [NumPts];
    //bool *RemovEIdx = new bool [NumPts];
    //memset(RemovEIdx,0,NumPts*sizeof(int));
    //memset(Overlap,0, NumPts*sizeof(int));

    if(CurrentListIndex >= 600) {
         LowboundSearch= CurrentListIndex-600;
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

            //cout<<ReproInx<<endl;
            AddtoMap(ReproInx, temp, V2Location, V2Frame);
        }
    }

    // ADD new points for triangulation//
    for (int i=0;i< NumPts ;i++)
    {
      if (Overlap[i]==-999)
      {
          vector<_2DFea> tempvec;
          _2DFea tempL,  tempR;
          
          tempL._2DPt = leftLocation[i];
          tempL.FrameNum = FrameNum-1;

          tempR._2DPt = RightLocation[i];
          tempR.FrameNum = FrameNum;

          tempvec.push_back(tempL);
          tempvec.push_back(tempR);

          AddtoMap(i,tempvec, V2Location, V2Frame);
      }
    }
}

void  _3DPt::PointRefinement(vector<v3_t> _3Dpts ,vector<vector<v2_t> >&V2Location, vector<vector<int> >&V2Frame,vector<int>& Overlap, vector<bool>& tempvector)
{
    int CurrentListIndex= _3DPt::Numof3Dpts();
    int LowboundSearch;
    int UpperboundSearch;

    if(CurrentListIndex >= 600) {
        LowboundSearch= CurrentListIndex-600;
        UpperboundSearch= CurrentListIndex;
    }

    else{
        LowboundSearch= 0;
        UpperboundSearch= CurrentListIndex;
    }

    // reset NewPt//
    for (int i= LowboundSearch;i<UpperboundSearch;i++)
        _3D[i].NewPt=0;


   for (int i=0;i<(int) Overlap.size();i++)
   {
     if ( Overlap[i] != -999) /* if overlapped point */ {

          //cout<<Overlap[i]<<endl;
          if ( ! tempvector[i])
          {
                //cout<<Overlap[i]<<endl;
                int idx = Overlap[i];
                // update 3D points
                _3D[idx].Point[0]=_3Dpts[i].p[0];
                _3D[idx].Point[1]=_3Dpts[i].p[1];
                _3D[idx].Point[2]=_3Dpts[i].p[2];

                // add last 2d location and 2d camera
                _2DFea temp;
                temp._2DPt = V2Location[i].back();
                temp.FrameNum = V2Frame[i].back();

                //cout<< "add overlapped point: "<< V2Frame[i].back()<<endl;
                //_3D[idx]._2D.back()._2DPt=V2Location[i].back();
                //_3D[idx]._2D.back().FrameNum=V2Frame[i].back();

                _3D[idx]._2D.push_back(temp);

                //cout<< "add overlapped point :";
                //cout<<_3D[idx]._2D.back().FrameNum<<endl;

                RemoveReproFlag(idx);
                RemoveReproIndx(idx);


                _3D[idx].NewPt= 1;
                tempvector[i]=true;
                // turn off flag and index //
                // Set new flag //
            }

          else {

             int idx = Overlap[i];
             tempvector[i]=true;
             RemoveReproFlag(idx);
             RemoveReproIndx(idx);
          }
       }
    }

    // create new points //

    for (int i=0;i< tempvector.size();i++){
        if ( Overlap[i] == -999){
          if ( ! tempvector[i])
                {
                  //int idx = Overlap[i];
                Pt3D temp;
                temp.Point[0]=_3Dpts[i].p[0];
                temp.Point[1]=_3Dpts[i].p[1];
                temp.Point[2]=_3Dpts[i].p[2];
                temp.NewPt= 1;

                int Size =(int) V2Location[i].size();

                for (int j=0; j< Size ;j++)
                  {
                    _2DFea Temp;
                    Temp._2DPt = V2Location[i][j];
                    Temp.FrameNum = V2Frame[i][j];
                    temp._2D.push_back(Temp);
                    temp.NewPt=1;
                }
               _3D.push_back(temp);
           }
        }
    }

    //for (int i=0;i<(int)NUM3D();i++)
    //    cout<<"adding 3D: "<<_3D[i].Point[0]<<" "<< _3D[i].Point[1]<<" "<< _3D[i].Point[2]<<endl;

}

//generate point map for bundle adjustment //
void _3DPt::MapGeneration(vector<v3_t>& _3Dpts ,vector<vector<v2_t> >&V2Location, vector<vector<int> >&V2Frame, vector<int>& SelectedIndex, int CurrentFrame, int NumCamera)
{

    // clean up the 3D points, 2D Location and Frame //

    vector<v3_t> temp3D;
    vector<vector<v2_t> > temp2dlocation;
    vector<vector<int> > tempframe;

    int _1stCamera = (CurrentFrame - NumCamera) +1;

    cout<<"Initial camera shift: "<<_1stCamera<<" "<<CurrentFrame<<endl;

    _3Dpts.swap(temp3D);
    V2Location.swap(temp2dlocation);
    V2Frame.swap(tempframe);

    // read 3d points //
    int CurrentListIndex= _3DPt::Numof3Dpts();
    int LowboundSearch;
    int UpperboundSearch;

    if(CurrentListIndex >= 600) {
        LowboundSearch= CurrentListIndex-600;
        UpperboundSearch = CurrentListIndex;
    }

    else{
        LowboundSearch= 0;
        UpperboundSearch= CurrentListIndex;
    }

  /*print out the frame number*/
  //for (int i= LowboundSearch;i<UpperboundSearch;i++){
  //    PrintNumFrame(i);
  //    cout<<endl;
  //}
  /*
  cout<<"first camera "<<_1stCamera<<" "<<CurrentFrame<<endl;

    for (int i= LowboundSearch;i<UpperboundSearch;i++){
         for (int j=0; j<_3D[i]._2D.size();j++)
         {
             cout<<_3D[i]._2D[j].FrameNum<<" ";
            }
         cout<<endl;
         }
 */
    for (int i= LowboundSearch;i<UpperboundSearch;i++){
        //if  (LatestCamera(i)== CurrentFrame)
        //cout<< InitialCamera(i)<< endl;

        if (InitialCamera(i)>= _1stCamera && LatestCamera(i) <= CurrentFrame)
        {

           _3Dpts.push_back(vec3D(i));

            V2Location.push_back(vector<v2_t>());
            V2Frame.push_back(vector<int>());

            V2Location[(int)V2Location.size()-1]=vec2D(i);
            V2Frame[(int)V2Frame.size()-1]=vecFrame(i);

            SelectedIndex.push_back(i);

        }
    }
}

void _3DPt::MapUpdate(vector<int> SelectedIdex, vector<v3_t>& _3Dpts, vector<size_t> RemoveIdx)
 {

     // update the 3D points here //
     //int numpts= (int) SelectedIdex.size();
     //int numpts=(int)_3Dpts.size();
     //for (int i=0;i < numpts;i++){
         //if (SelectedIdex[i] !=9999){
         //    int index =SelectedIdex[i];
                 // update 3D points //
        //     Set3Dpt(i, _3Dpts[i]);
     //   }
    //}

    //remove the outliers//
     int numpts=(int)_3Dpts.size();
     for (int i=0;i<numpts;i++){

         int index = SelectedIdex[i];
         Set3Dpt(index, _3Dpts[i]);
     }

     int shift=0;
     
     cout<<NUM3D()<<endl;

     for (int i=0;i<RemoveIdx.size();i++) {
         size_t index = RemoveIdx[i]-shift;
          cout<<"Remov Index "<<RemoveIdx[i] << "index "<<index<<endl;
         _3D.erase(_3D.begin()+(index));
         shift++;
        }




     //for (int i=0;i<(int)NUM3D();i++)
     //    cout<<_3D[i].Point[0]<<" "<< _3D[i].Point[1]<<" "<< _3D[i].Point[2]<<endl;
 }