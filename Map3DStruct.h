//
//  Map3DStruct.h
//  OpenCVtracking
//
//  Created by C-HChang on 11/19/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#ifndef __OpenCVtracking__Map3DStruct__
#define __OpenCVtracking__Map3DStruct__


#include <stdio.h>

# include <vector>
# include <iostream>
# include "matrix.h"
# include "vector.h"
# include "Relative_Pose.h"

using namespace std;

#define VALID -99999
#define NonVal -1

typedef  struct
{
    v2_t _2DPt;
    int FrameNum;
} _2DFea;

typedef struct
{
    double Point[3];
    vector<_2DFea> _2D;
    //vector<int> framelist;
    bool flagBAdj=0;
    bool flagRepro=0;
    bool NewPt=0;
    int  RepIndx= VALID;
    bool removeIdx=0;

} Pt3D;

void inline AddtoMap(int idx, vector<_2DFea> input ,vector<vector<v2_t> >& V2Location , vector<vector<int> >& V2Frame ){
    /*Initialization of 2D array */

    //V2Location.push_back(vector<v2_t>());
    //V2Frame.push_back(vector<int>());

    int idxLocation = (int) V2Location.size();
    int idxFrame = (int) V2Frame.size();

    for (int i=0;i< input.size();i++)
    {
        cout<< idx <<endl;
        V2Location[idx].push_back(input[i]._2DPt);
        V2Frame[idx].push_back(input[i].FrameNum);
    }
}

class _3DPt
{


public:
   
    vector<Pt3D> _3D;

    //CameraPose ScameraPose ;

    void Initial(int NumPts, vector<v3_t>pt, vector<v2_t> leftLocation, vector<v2_t> RightLocation, int FrameNumber);
   
    _3DPt();
   
    void Initialization(v3_t pt, v2_t LeftLocation, v2_t RightLocation, int FrameNumber);

    void ConnectNewFrame(int NumPts, v2_t* leftLocation, v2_t* RightLocation, int FrameNumber, vector<v2_t>&Reproject2D, vector<v3_t>& Project3D);

    //void _3DPtGeneration(int NumPts , int FrameNum ,v2_t* leftLocation, v2_t* RightLocation);
    //void _3DPtGeneration( int NumPts , int FrameNum,  v2_t* leftLocation, v2_t* RightLocation,  vector<vector<v2_t> >&V2Location, vector<vector<int> >&V2Frame);

    void _3DPtGeneration( int NumPts , int FrameNum,  v2_t* leftLocation, v2_t* RightLocation,  vector<vector<v2_t> >&V2Location, vector<vector<int> >&V2Frame, vector<int>& Overlap);

    void PointRefinement(vector<v3_t> _3Dpts ,vector<vector<v2_t> >&V2Location,vector<vector<int> >&V2Frame,vector<int>& Overlap, vector<bool>& tempvector);

    void MapGeneration(vector<v3_t>& _3Dpts ,vector<vector<v2_t> >&V2Location, vector<vector<int> >&V2Frame,vector<int>& SelectedIndex,  int CurrentFrame, int NumCamera);

    inline int NumofCamera(int i)
    {
             return((int)_3D[i]._2D.size());
         }

    inline int Numof3Dpts() {
        return((int)_3D.size());
    }
    inline int LatestCamera(int i) {
        return((int)_3D[i]._2D.back().FrameNum);
    }
    inline int InitialCamera(int i){
        return((int)_3D[i]._2D.front ().FrameNum);
    }
    
    inline v3_t Read3DPoint(int i){
        v3_t temp;
        temp.p[0]= _3D[i].Point[0]; temp.p[1]= _3D[i].Point[1]; temp.p[2]= _3D[i].Point[2];
        return(temp);
    }
    inline bool ReadReproFlag(int i) {
        return(_3D[i].flagRepro);
    }
    inline int ReadReproIndx(int i) {
        return( _3D[i].RepIndx);
    }
    inline void ActivateReproFlag(int i) {
             _3D[i].flagRepro = true;
    }
    inline void ActivateReproIndx(int i, int Indx) {
             _3D[i].RepIndx = Indx;
    }
    inline void ActivateNewFlag(int i) {
             _3D[i]. NewPt = true;
    }   
    inline void RemoveReproFlag(int i) {
             _3D[i].flagRepro = false;
    }
    inline void RemoveReproIndx(int i) {
             _3D[i].RepIndx = VALID;
    }
    inline void RemoveNewFlag(int i) {
             _3D[i]. NewPt = false;
    }   
    inline vector<_2DFea> Read2DPt(int i) {
            return(_3D[i]._2D);
    } 

    inline int NUM3D() {return((int)_3D.size());}
    
    inline int NUMFRAME(int i)
    {
        return((int)_3D[i]._2D.size());
    }

    inline v3_t vec3D (int i){
            v3_t pt;
            pt.p[0]= _3D[i].Point[0];
            pt.p[1]= _3D[i].Point[1];
            pt.p[2]= _3D[i].Point[2];
            return(pt);
    }

    inline vector<int> vecFrame (int i)
    {
           vector<int> tempframe;
           int size = NUMFRAME(i);
           
           for (int j=0;j< size;j++) 
             tempframe.push_back(_3D[i]._2D[j].FrameNum);

        return(tempframe);
    }
    // output 2D vector of 2d features //
    inline vector<v2_t> vec2D (int i)
    {
           vector<v2_t> templocation;
           int size = NUMFRAME(i);
           for (int j=0;j< size;j++) 
            templocation.push_back(_3D[i]._2D[j]._2DPt);
        
        return(templocation);
    }


    inline void PrintNumFrame(int i )
    {
        int size = NUMFRAME(i);
        for (int j=0;j<  size ;j++) {
             cout<<_3D[i]._2D[j].FrameNum<<" ";
        }
    }
};



#endif /* defined(__OpenCVtracking__Map3DStruct__) */
