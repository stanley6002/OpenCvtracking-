//
//  featurepointsmap.cpp
//  OpenCVtracking
//
//  Created by C-HChang on 5/17/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#include "featurepointsmap.h"
#include <math.h>

#define NUM -99999
using namespace std;

FeaMapPoints::FeaMapPoints ()
{
    CurrentListIndex=0;
    PreviousListIndex=0;
}
FeaMapPoints::~FeaMapPoints ()
{
}
void FeaMapPoints::ConnectedFMVideoSequence(v2_t* Connected_pts /* current frame left points*/  ,v2_t* Current_pts /*current new frame points*/, int Numpts , int FrameNumber)
{
    //int Previous_ptsize   =   (int)Previous_pts.size();
    int ConnectedPtsize   =   Numpts;

    // serach frame list in here //
    int LowboundSearch;
    int UpperboundSearch;

    if(PreviousListIndex> 400)
    {
        LowboundSearch= CurrentListIndex-400;
        UpperboundSearch= CurrentListIndex;
    }

    else{
        LowboundSearch= 0;
        UpperboundSearch= CurrentListIndex;
    }

    int Memsize = UpperboundSearch-LowboundSearch;

    //int* StackIdx = new int [Memsize];
    int* tempPrevious = new int [Memsize];
    int* tempCurrent  = new int [ConnectedPtsize];

    //int framelist_size= (int) FM_v2_frame.size();
    //memset(StackIdx, 0, Memsize*sizeof(int));

    memset(tempPrevious, 0, Memsize*sizeof(int));
    memset(tempCurrent,  0, ConnectedPtsize*sizeof(int));

    //for (int i=0;i< Memsize;i++)
    //    TriIndex.push_back(NULL);

    int numreprojection =0;

    for (int i=0; i< Memsize; i++)
    {
        int shiftlowbound = i+LowboundSearch;
        int FrameRowsize = (int) FM_v2_frame[i].size();

        if (FM_v2_frame[shiftlowbound][FrameRowsize-1] == FrameNumber-1)
        {
        if (tempPrevious[i] != NUM)
           {
                int rowsize = (int) FM_v2_location[shiftlowbound].size();
                int x = FM_v2_location[shiftlowbound][rowsize-1].p[0];
                int y = FM_v2_location[shiftlowbound][rowsize-1].p[1];

                for (int j=0; j< ConnectedPtsize; j++)
                 {
                    if (tempCurrent[j] != NUM)
                    {
                        int x_m =  (int) Connected_pts[j].p[0];
                        int y_m =  (int) Connected_pts[j].p[1];

                        if (sqrt(((x-x_m)*(x-x_m))+((y-y_m)*(y-y_m)))<=1.414)
                        {
                            numreprojection++;
                            //StackIndex[i]=j;
                            v2_t P;
                            P.p[0]= shiftlowbound;   // add the framelist location //
                            P.p[1]= j;               // correspondence number //
                            StackIndex.push_back(P);
                            tempCurrent[j] =NUM;
                            tempPrevious[i]=NUM;

                            TriIndex.push_back(shiftlowbound);

                            break;
                        }
                    }
               }
           }
        }
     }
    //cout<<"number of reprojection:  " <<numreprojection<<endl;
    CreateFMFeatureTrack(tempCurrent, ConnectedPtsize, Connected_pts, Current_pts , FrameNumber);
    CollectFMFeatureTrackProjectPts(Current_pts, FrameNumber);

    //delete [] StackIdx;
    
    delete [] tempCurrent;
    delete [] tempPrevious;
}

// create feature track with new upcoming frame //
// Connectec_pts -> first points
// Current_pts -> second points
void FeaMapPoints::CreateFMFeatureTrack(int* tempCurrent, int ConnectedPtsize, v2_t* Connected_pts/* current left points*/, v2_t* Current_pts/*current new points*/, int FrameNumber)
{
    for (int i =0;i<ConnectedPtsize ;i++)
    {
        if(tempCurrent[i] != NUM)
        {
            FM_v2_frame.push_back(vector<int>());
            int size_frame =(int)FM_v2_frame.size();

            //cout<<"size of frame list "<<size_frame<<endl;
            FM_v2_frame[size_frame-1].push_back(FrameNumber-1);
            FM_v2_frame[size_frame-1].push_back(FrameNumber);

            //initialized
            FM_v2_location.push_back(vector<v2_t>());
            int size_2Dlocation =(int)FM_v2_location.size();

            FM_v2_location[size_2Dlocation-1].push_back(Connected_pts[i]);
            FM_v2_location[size_2Dlocation-1].push_back(Current_pts[i]);

            TriIndex.push_back((int) FM_v2_frame.size()-1);

        }
    }
}
void FeaMapPoints::CollectFMFeatureTrackProjectPts(v2_t* Current_pts, int FrameNum)
{
     int size_= (int) StackIndex.size();

    for (int i=0;i<size_;i++)
    {
            int indexi  = StackIndex[i].p[0];
            int indexj  = StackIndex[i].p[1];

            FM_v2_location[indexi].push_back(Current_pts[indexj]);    // add connected feature point to existing list and collect 3D->2d POINTS
            FM_v2_frame[indexi].push_back(FrameNum);
            FM_v2ReprojectPts.push_back(Current_pts[indexj]);    // collect 2D reprojection pts
            FM_v3ProjectionPts.push_back(FM_3DLocation[indexi]);           // collect
        }
    // Update the number of reprojection points
    this->NumReproject = (int) FM_v3ProjectionPts.size();
    //cout<< " Projection size "<<  FM_v3ProjectionPts.size()<<endl;
    //cout<< " ReProjection size "<< FM_v2ReprojectPts.size()<<endl;
}