//
//  CameraPoseRefinement.h
//  Optical_flow 
//
//  Created by chih-hsiang chang on 4/3/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
    
#include "matrix.h"
#include "vector.h"
    
    void CameraRotRefine(int num_points, v3_t *points, v2_t *projs , double *R, double *T, double* Kmatrix);
    
    static void Sing_camera_refine(int num_points, v3_t *points, v2_t *projs, double*R, double*T, double*Kmatrix);
    
    static void CameraReprojectionError (const int *m, const int *n, 
                                         double *x, double *fvec, int *iflag);
    
    static void CameraProjectPoint(double *aj, double *bi, 
                                   double *xij);
    
    //static void Sing_sfm_project( 
    //                             double *w /*Rot parameters*/ , double * b /*3Dpts*/ , double *p/*projection*/);
    
    void Sing_rot_update(double *R, double *w, double *Rnew);
    
    static void projection_residual(const int *m, const int *n, double *x, double *fvec, double *iflag) ;
    //    
    //    int find_projection_3x4_nonlinear(int num_pts, v3_t *points, v2_t *projs, 
    //                                      double *Pin, double *Pout) ;
    //    
    //    int find_projection_3x4_ransac(int num_pts, v3_t *points, v2_t *projs, 
    //                                   double *P, 
    //                                   int ransac_rounds, double ransac_threshold) ;    
    //    void FixIntrinsics(double *P, double *K, double *R, double *t) ;
    //    void CameraParameter_Process(double*P, double *Rinit, double *tinit,double *K/* frame number*/ );
    
    
#ifdef __cplusplus
}
#endif