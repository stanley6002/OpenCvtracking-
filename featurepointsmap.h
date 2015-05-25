//
//  featurepointsmap.h
//  OpenCVtracking
//
//  Created by C-HChang on 5/17/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#ifndef __OpenCVtracking__featurepointsmap__
#define __OpenCVtracking__featurepointsmap__

#include <stdio.h>

# include <vector>
# include <iostream>
# include "matrix.h"
# include "vector.h"

using namespace std;

class FeaMapPoints
{
    //friend class CameraPose;
    //friend class EpipolarGeometry;
public:

    int CurrentListIndex;
    int PreviousListIndex;

    vector<v2_t> StackIndex;  // for update video connected points
    vector<int> TriIndex;     // for updating triangulation list

    int NumReproject;
    int FrameNumber;

    int List_Index;

    vector<v3_t> FM_3Dpts;

    vector<v2_t> FM_leftPts;
    vector<v2_t> FM_rightPts;

    // store find 3D position //
    vector<v3_t> FM_3DLocation;

    vector<v2_t> FM_v2ReprojectPts;
    vector<v3_t> FM_v3ProjectionPts;

    vector<vector<v2_t> > FM_v2_location;   // show 2D point locations
    vector<vector<int> >  FM_v2_frame;      // show frame list

    inline void LoadFMFeatureList(int FrameNum)
    {
        //load  number of 2D Points to list
        int size_= (int) this-> FM_leftPts.size();
        if (size_ != (int) FM_rightPts.size())
        {
            cout<<"size different;program exit"<<endl;
            exit(1);
        }

        for(int i=0;i< size_;i++)
        {
            FM_v2_location.push_back(vector<v2_t>());
            FM_v2_location[i].push_back(FM_leftPts[i]);
            FM_v2_location[i].push_back(FM_rightPts[i]);

            int Numofview = FrameNum;

            FM_v2_frame.push_back(vector<int>());

            for (int j=0;j<Numofview;j++)
            {
                FM_v2_frame[i].push_back(j);
            }

        }
        
        // update ListIndex for searching and connect frame list
        if (PreviousListIndex==0)
        {
            CurrentListIndex= (int)FM_v2_location.size();
        }
        else
        {
            PreviousListIndex= CurrentListIndex;
            CurrentListIndex+= (int)FM_v2_location.size();
        }

    }

    inline void LoadFMv2Pts(vector<v2_t>& left_pts, vector<v2_t>& right_pts)
    {
        FM_leftPts.insert(FM_leftPts.end() , left_pts.begin() , left_pts.end() );
        FM_rightPts.insert(FM_rightPts.end() , right_pts.begin() , right_pts.end() );
    }

    inline void LoadFMv3Pts(vector<v3_t>& V3Dpts)
    {
        FM_3DLocation. insert(FM_3DLocation.end(), V3Dpts.begin(), V3Dpts.end());
    }

    void ConnectedFMVideoSequence( v2_t* Connected_pts /*current new frame*/, v2_t* Current_pts, int Numpts , int FrameNumber);

    void CreateFMFeatureTrack(int* tempCurrent, int ConnectedPtsize, v2_t* Connected_pts/* current left points*/,
                              v2_t* Current_pts/*current new points*/, int FrameNumber);

    void CollectFMFeatureTrackProjectPts(v2_t* Current_pts, int FrameNum);

    FeaMapPoints();
    ~FeaMapPoints();
};

#endif /* defined(__OpenCVtracking__featurepointsmap__) */
