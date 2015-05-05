/* 
 *  Copyright (c) 2008  Noah Snavely (snavely (at) cs.washington.edu)
 *    and the University of Washington
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */


#ifndef __included_typedefine_h
#define __included_typedefine_h

#include "vector.h"
#include <vector>

using namespace std;


typedef struct 
{
    vector <v3_t>R_pts;
    vector <v3_t>L_pts;

}  F_refined;

typedef struct 
{
    vector<v3_t>R_pts;
    vector<v3_t>L_pts;
    
} matched;

typedef struct 
{
    double p[9];
    
} F_key_matrix;

typedef struct 
{
    vector<v3_t>left_input;
    vector<v3_t>right_input;
} p3_t ;

typedef struct
{
    vector<v3_t> threeDpt;
} threeDpt;  

typedef struct 
{
    double R_matrix[9];
    double T_matrix[3];

} cameramatrix;

typedef struct 
{

vector <v3_t> pts; 

}V3D_position;

typedef struct 
{    

    vector <v3_t> Rs; 

}R_reserved;
 typedef struct 
{
    vector<int> ThreeDindex;
    vector<int> TwoDindex;
    
} reserved_index;
typedef struct
{
double ELEMENT [9];
} Rotation_matrix;
typedef struct
{
double ELEMENT [3];
} Translation_matrix;
typedef struct 
{
 double ELEMENT[9];
} Intrinsic_matrix;

#endif