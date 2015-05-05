
#ifndef __epipolar_h__
#define __epipolar_h__

#ifdef __cplusplus
extern "C" {
#endif

#include "vector.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <float.h>
#include "qsort.h"
#include "svd.h"
#include "matrix.h"
#include "5point.h"
#include "vector.h"
#include "triangulate.h"
    
bool  checkna (double tao);

int estimate_fmatrix_ransac_matches(int num_pts, v3_t *a_pts, v3_t *b_pts, 
                                        int num_trials, double threshold, 
                                        double success_ratio,
                                    int essential, double *F); 

int estimate_fmatrix_linear(int num_pts, v3_t *r_pts, v3_t *l_pts, 
                            int essential,
                            double *Fout, double *e1, double *e2);
int closest_rank2_matrix(double *Fin, double *Fout, double *U, double *VT);
//int closest_rank2_matrix_ssv(double *Fin, double *Fout, 
//                             double *U, double *VT);
int svd3_driver(double *A, double *U, double *S, double *VT); 

double fmatrix_compute_residual(double *F, v3_t r, v3_t l);

void fmatrix_residuals(int *m, int *n, double *x, double *fvec, int *iflag);
//void fmatrix_residuals(const int *m, const int *n, double *x, double *fvec,double *iflag);

void refine_fmatrix_nonlinear_matches(int num_pts, v3_t *r_pts, v3_t *l_pts, 
                                      double *F0, double *Fout); 

int EstimatePose5Point(v2_t* k1, 
                           v2_t* k2, 
                           int matches, 
                           int num_trials, double threshold, 
                           double *K1, double *K2, 
                           double *R, double *t); 

v3_t Triangulate(v2_t p, v2_t q, double *camera_1R,double *camera_1t,double *camera_2R, double *camera_2t,  
                     double proj_error, bool in_front, double angle,
                     bool explicit_camera_centers, double *K1, double *K2);
  
double ComputeRayAngle(v2_t p, v2_t q, double *K1_inv, double *K2_inv, double*camera_1R, double *camera_2R,double *camera_1t, double *camera_2t);

//bool CheckCheirality(v3_t p, double * camera_R, double *camera_t); 
//v2_t UndistortNormalizedPoint(v2_t p);
    
double F_matrix_residule(double *F,v3_t r, v3_t l);

void refine_fmatrix_nonlinear_Pro(int num_pts, double* F0, double *Fout, double *epipole,v3_t *r_pts, v3_t *l_pts);

void refine_nonlinear_projection(int num_pts, v3_t *r_pts, v3_t *l_pts, 
                                     double *P_matrix, double *Fout);
//    void Pmatrix_residuals(const int *m, const int *n, double *x, double *fvec, double *iflag);
//    double Pmatrix_compute_residual(double *P, v3_t _2Dpoint, int i, int j, int index); 
double fmatrix_compute_distance(double *F, v3_t r, v3_t l); 

v2_t UndistortNormalizedPoint(v2_t p); 

float Apical_Angle( v2_t* l_pt, v2_t* r_pt, double *R, double *t,double *K , const int size_);

float KDE_estimation(double * Tao, const int size_, const int Nbins);

#ifdef __cplusplus
}
#endif

#endif