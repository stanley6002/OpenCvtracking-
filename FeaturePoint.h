//
//  FeaturePoint.h
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/27/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//



# include <vector>
# include <iostream>
# include "matrix.h"
# include "vector.h"

using namespace std;


class FeaturePts 
{   
    //friend class CameraPose;
    //friend class EpipolarGeometry;
public:
    vector<v3_t> m_3Dpts; 
    
    vector<v2_t> m_leftPts;  
    vector<v2_t> m_rightPts;
    
    // store find 3D position //
    vector<v3_t> _3DLocation;
    
    vector<v2_t> mv2ReprojectPts;
    vector<v3_t> mv3ProjectionPts;
    
    int NumReproject;
    int FrameNumber;
    
    vector<vector<v2_t> > mv2_location;   // show 2D point locations
    vector<vector<int> >  mv2_frame;      // show frame list 

    
    
    // temporary storage //

    
    //  Add Frame Sequences at following:
    //
    //  Frame : RIGHT->  LEFT  ->(incoming third view) -> RIGHT 
    //          First   Second                            Third
    //
    //
    //
    //  Feature Point list (column: frame number across  ; row number: number of points  )     
    //  Feature Point List   
    //                                           
    //     p11   p12   p13   three common points                                pnl  
    //     p21   p22   p23   three common points                                pnr
    //     P31   p32   p33   three common points
    //
    //           number
    //            of 
    //           points
    //
    //  new incoming points add  at the end of list 
    //  For example 
    //  add  two new points
    //  
    //  pn2     pn3
    //  pn+1 2  pn+1 3 
    //  three common points                          two common points
    //  across three consectutive frame          across two consecutive 
    //  temporary storage //
    
  
    
    inline void LoadFeatureList(int FrameNum)
    {
       
        int size_= (int) this-> m_leftPts.size(); 
        
        for(int i=0;i< size_;i++)
        {
            mv2_location.push_back(vector<v2_t>());
            mv2_location[i].push_back(m_leftPts[i]);
            mv2_location[i].push_back(m_rightPts[i]);   
            
            int Numofview = FrameNum;
            
            mv2_frame.push_back(vector<int>());
            for (int j=0;j<Numofview;j++)
             {
               mv2_frame[i].push_back(j);
              }   
           
        }
        
      //  InitializedFirstTwoFrameList(size_);
    }
    
    
    /*inline void InitializedFeaturePtsList(int size_)
    {
    
        for(int i=0;i< size_;i++)
        {
            mv2_location.push_back(vector<v2_t>());
            mv2_location[i].push_back(m_leftPts[i]);
            mv2_location[i].push_back(m_rightPts[i]);  
        }
    }*/
    
    /*inline void InitializedFirstTwoFrameList(int size_)
    {
        int Numofview =2;
        for (int i =0; i<size_;i++)
        {
          mv2_frame.push_back(vector<int>());
             for (int j=0;j<Numofview;j++)
          {
            mv2_frame[i].push_back(j);
            }
        }
    }*/
    
    inline void Loadv2Pts(vector<v2_t>& left_pts, vector<v2_t>& right_pts)
    {
        m_leftPts.insert(m_leftPts.end() , left_pts.begin() , left_pts.end() );
        m_rightPts.insert(m_rightPts.end() , right_pts.begin() , right_pts.end() );
    }
    
    inline void Loadv3Pts(vector<v3_t>& V3Dpts)
    {
        m_3Dpts. insert(m_3Dpts.end(), V3Dpts.begin(), V3Dpts.end());  
    }  
    
    void ConnectedVideoSequence(vector<v2_t> Previous_pts   /*previous frame*/, v2_t* Connected_pts, v2_t* Current /*current new frame*/, int Numpts, int FrameNumber);
   
    void CreateFeatureTrack(int* tempCurrent, int ConnectedPtsize, v2_t* Connected_pts, v2_t* Current_pts,int FrameNumber);
    
    //void CollectFeatureTrackProjectPts(int Previous_ptsize , v2_t* Current_pts, int FrameNum);
    void CollectFeatureTrackProjectPts(int Previous_ptsize , v2_t* Current_pts, int FrameNum, int* StackIdx);
   
    void UpdatedFeatureTrack(vector<v2_t>& left_pts, vector<v2_t>& right_pts, vector<v3_t>& V3Dpts, int FrameNum);
    
    void CleanFeatureTrack();
    
    void PointRefinement(vector<v3_t>& Tempv3Dpts, vector <bool> tempvector);
    
    FeaturePts ();
    ~ FeaturePts ();

private:
    //int* StackIndex;
    
   
    
};
