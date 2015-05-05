//
//  miscul.h
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/19/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//


#include "typedefine.h"
#include "Opencvheaders.h"

inline void CvPoint32fvectorv3_t(const std::vector<CvPoint2D32f> match_query , const std::vector<CvPoint2D32f> match_train ,  v3_t*r_pt , v3_t*l_pt , int size_)
{
  
    for(int i=0 ; i< size_;i++) 
    {
        //pt_new_query[i].x = match_query[i].x;
        //pt_new_query[i].y = match_query[i].y;
        
        //pt_new_train[i].x = match_train[i].x;
        //pt_new_train[i].y = match_train[i].y;
        
        r_pt[i].p[0]=match_query[i].x;
        r_pt[i].p[1]=match_query[i].y;
        r_pt[i].p[2]=1.0;
        
        l_pt[i].p[0]=match_train[i].x;
        l_pt[i].p[1]=match_train[i].y;
        l_pt[i].p[2]=1.0;
        
    }


}