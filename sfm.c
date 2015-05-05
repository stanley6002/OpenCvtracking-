
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
 *
 *
 * SBA routine modified  sfm_project_rd for our reprojection error;
 * "mean projection error calaulation" 
 * with know K matrix element and undistrotion  element. 
 */
 /* sfm-driver.c */
/* Driver for sfm routines */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sba.h"
#include "matrix.h"
#include "vector.h"
#include "sfm.h"

#ifdef WIN32
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#endif /* WIN32 */

// #define COLIN_HACK
#define TEST_FOCAL

static double *global_last_ws = NULL;
static double *global_last_Rs = NULL;

typedef struct 
{
    int num_cameras;               /* Number of cameras */
    int num_points;                /* Number of points */
    int num_params_per_camera;     /* Number of parameters for each camera */
    int est_focal_length;          /* Should the focal length be estimated? */
    camera_params_t global_params;
    camera_params_t *init_params;  /* Initial camera parameters */
    v3_t *points;
} 
sfm_global_t;

static void *safe_malloc(int n, char *where)
{
    void *mem = malloc(n);
    
    if (mem == NULL) {
	if (where) {
	    printf("[safe_malloc] Error allocating %d bytes "
		   "of memory at %s\n", n, where);
	} else {
	    printf("[safe_malloc] Error allocating %d bytes of memory\n", n);
	}

	fflush(stdout);
	exit(1);
    }

    return mem;
}

/* Compute an updated rotation matrix given the initial rotation (R)
 * and the correction (w) */
void rot_update(double *R, double *w, double *Rnew) 
{
    double theta, sinth, costh, n[3];
    double nx[9], nxsq[9];
    double term2[9], term3[9];
    double tmp[9], dR[9];

    double ident[9] = 
	{ 1.0, 0.0, 0.0,
	  0.0, 1.0, 0.0,
	  0.0, 0.0, 1.0 };

    theta = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);

    if (theta == 0.0) {
	memcpy(Rnew, R, sizeof(double) * 9);
	return;
    }

    n[0] = w[0] / theta;
    n[1] = w[1] / theta;
    n[2] = w[2] / theta;

    nx[0] = 0.0;   nx[1] = -n[2];  nx[2] = n[1];
    nx[3] = n[2];  nx[4] = 0.0;    nx[5] = -n[0];
    nx[6] = -n[1]; nx[7] = n[0];   nx[8] = 0.0;

    matrix_product33(nx, nx, nxsq);

    sinth = sin(theta);
    costh = cos(theta);

    matrix_scale(3, 3, nx, sinth, term2);
    matrix_scale(3, 3, nxsq, 1.0 - costh, term3);

    matrix_sum(3, 3, 3, 3, ident, term2, tmp);
    matrix_sum(3, 3, 3, 3, tmp, term3, dR);

    matrix_product33(dR, R, Rnew);
}

void sfm_project_rd(camera_params_t *init, double *K, double *k,
                    double *R, double *dt, double *b, double *p)
{
    double *t;    
    double tnew[3];
    double b_cam[3];
    
    t = init->t;
    
    tnew[0] = dt[0];
    tnew[1] = dt[1];
    tnew[2] = dt[2];

    double b2[3];
    b2[0] = b[0] - tnew[0];
    b2[1] = b[1] - tnew[1];
    b2[2] = b[2] - tnew[2];
    matrix_product331(R, b2, b_cam);
    p[0] = -b_cam[0] * K[0] / b_cam[2];
    p[1] = -b_cam[1] * K[0] / b_cam[2];
}


static void sfm_project_point3(int j, int i, double *aj, double *bi, 
			       double *xij, void *adata)
{
    sfm_global_t *globs = (sfm_global_t *) adata;
    
    double K[9] = 
	{ 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0 
    };
    
    double *w, *dt, *k;
    
    /* Compute intrinsics */
    if (!globs->est_focal_length)
    {
      K[0] = K[4] = globs->init_params[j].f; // globs->global_params.f;
    } 
    else 
    {

      K[0] = K[4] = aj[6] / globs->init_params[j].f_scale;
    }
    
    /* Compute translation, rotation update */
    dt = aj + 0;
    w = aj + 3;
    
    if (globs->est_focal_length)
        k = aj + 7;
    else
        k = aj + 6;
    
    if (w[0] != global_last_ws[3 * j + 0] ||
        w[1] != global_last_ws[3 * j + 1] ||
        w[2] != global_last_ws[3 * j + 2]) 
    {
        
        rot_update(globs->init_params[j].R, w, global_last_Rs + 9 * j);
        global_last_ws[3 * j + 0] = w[0];
        global_last_ws[3 * j + 1] = w[1];
        global_last_ws[3 * j + 2] = w[2];
    }
    
    sfm_project_rd(globs->init_params + j, K, k, global_last_Rs + 9 * j, 
                   dt, bi, xij);
}

static void sfm_project_point3_mot(int j, int i, double *aj, 
                                   double *xij, void *adata)
{
    sfm_global_t *globs = (sfm_global_t *) adata;
    double *b = globs->points[i].p;

    sfm_project_point3(j, i, aj, b, xij, adata);    
}

#define SBA_V121

void run_sfm(int num_pts, int num_cameras, int ncons,   /* num_pts = 3D points*/
             char *vmask,
             double *projections,
             int est_focal_length,
             camera_params_t *init_camera_params,
             v3_t *init_pts, 
             int use_constraints, 
             int use_point_constraints,
             int fix_points,
             int NumIteration,
             double eps2,
             double *Vout, 
             double *Sout,
             double *Uout, double *Wout
             /* size num_cameras ** 2 * cnp * cnp */)
{
    #define VERBOSITY 3
    int cnp;
    double *params;
    double opts[6]; // opts[5];
    double info[10];
    int i, j, base;
    int num_camera_params, num_pt_params, num_params;

    sfm_global_t global_params;
    camera_constraints_t *constraints = NULL;
    point_constraints_t *point_constraints = NULL;
    
    const double f_scale = 0.001 ;
    
    if (est_focal_length)
	cnp = 7;
    else
	cnp = 6;

    num_camera_params = cnp * num_cameras;
    num_pt_params = 3 * num_pts;
    num_params = num_camera_params + num_pt_params;

    params = (double *) safe_malloc(sizeof(double) * num_params, "params");
    
    if (use_constraints) 
    {
        constraints = 
	    (camera_constraints_t *) 
        malloc(num_cameras * sizeof(camera_constraints_t));	
        
        for (i = 0; i < num_cameras; i++) 
        {
            constraints[i].constrained = (char *) malloc(cnp);
            constraints[i].constraints = 
            (double *) malloc(sizeof(double) * cnp);
            constraints[i].weights = (double *) malloc(sizeof(double) * cnp);
            
            memcpy(constraints[i].constrained, 
                   init_camera_params[i].constrained, cnp * sizeof(char));
            memcpy(constraints[i].constraints, 
                   init_camera_params[i].constraints, cnp * sizeof(double));
            memcpy(constraints[i].weights,
                   init_camera_params[i].weights, cnp * sizeof(double));
            
#ifdef TEST_FOCAL
            if (est_focal_length) 
            {
                constraints[i].constraints[6] *= f_scale;
                constraints[i].weights[6] *= (1.0 / (f_scale * f_scale));
                // constraints[i].constraints[6] *= 1;
                // constraints[i].weights[6] *= (1.0 / (0.001 * 0.001));
            }
            
           #endif
        }
    }
    
    if (use_point_constraints) 
    {
        point_constraints = 
	    (point_constraints_t *) 
        malloc(num_pts * sizeof(point_constraints_t));
        
        for (i = 0; i < num_pts; i++)
        {
            if (Vx(init_pts[i]) == 0.0 &&
                Vy(init_pts[i]) == 0.0 &&
                Vz(init_pts[i]) == 0.0)
            {
                
                point_constraints[i].constrained = 0;
                point_constraints[i].constraints[0] = 0.0;
                point_constraints[i].constraints[1] = 0.0;
                point_constraints[i].constraints[2] = 0.0;
                point_constraints[i].weight = 0.0;
                
            } 
            else 
            {
                // printf("[run_sfm] Constraining point %d\n", i);
                point_constraints[i].constrained = 1;
                point_constraints[i].weight = 0.0;
                point_constraints[i].constraints[0] = Vx(init_pts[i]);
                point_constraints[i].constraints[1] = Vy(init_pts[i]);
                point_constraints[i].constraints[2] = Vz(init_pts[i]);
            }
        }
    }

    
    
    
    /* Fill global param struct */
    global_params.num_cameras = num_cameras;
    global_params.num_points = num_pts;
    global_params.num_params_per_camera = cnp;
    global_params.est_focal_length = est_focal_length;
    global_params.init_params = init_camera_params;
    
    global_last_ws = 
	safe_malloc(3 * num_cameras * sizeof(double), "global_last_ws");
    
    global_last_Rs = 
	safe_malloc(9 * num_cameras * sizeof(double), "global_last_ws");
    
    global_params.points = init_pts;
    
    for (i = 0; i < num_cameras; i++) 
    {
        
        global_last_ws[3 * i + 0] = 0.0;
        global_last_ws[3 * i + 1] = 0.0;
        global_last_ws[3 * i + 2] = 0.0;
        
        memcpy(global_last_Rs + 9 * i, 
               init_camera_params[i].R, 9 * sizeof(double));
    }
  
    /* Fill parameters */
    for (j = 0; j < num_cameras; j++) 
    {
        int c = 0;
        init_camera_params[j].f_scale = f_scale;       
        params[cnp * j + 0] = init_camera_params[j].t[0]; 
        params[cnp * j + 1] = init_camera_params[j].t[1]; 
        params[cnp * j + 2] = init_camera_params[j].t[2]; 
        params[cnp * j + 3] = 0.0;
        params[cnp * j + 4] = 0.0;
        params[cnp * j + 5] = 0.0;
    
    /*Initialized focal length*/ 
        
	if (est_focal_length) 
    {
        /* Focal length is initial estimate */
         params[cnp * j + 6] = init_camera_params[j].f * f_scale;
         c = 7;
	} 
    else
    {
         c = 6;
    }
        
    }

    base = num_camera_params;
    for (i = 0; i < num_pts; i++) 
    {
        params[base + 3 * i + 0] = Vx(init_pts[i]);
        params[base + 3 * i + 1] = Vy(init_pts[i]);
        params[base + 3 * i + 2] = Vz(init_pts[i]);
    }

        opts[0] = 1.0e-3;
        opts[1] = 1.0e-12; // 1.0e-15;
        opts[2] = 1.0e-12;
        opts[3] = 1.0e-12;
        opts[4] = 0.0;
        opts[5] = 4.0e-10; // change this back to opts[4] for sba v1.2.1

    if (fix_points == 0) 
    {
            sba_motstr_levmar(num_pts, num_cameras, ncons, 
                              vmask, params, cnp, 3, projections,NULL, 2, 
                              //remove NULL in prev line for sba v1.2.1
                              sfm_project_point3, NULL, 
                              (void *) (&global_params),
                              NumIteration, VERBOSITY, opts, info,
                              use_constraints, constraints,
                              use_point_constraints,
                              point_constraints, Vout, Sout, Uout, Wout);
    }
    else
    {
            sba_mot_levmar(num_pts, num_cameras, ncons, 
                            vmask, params, cnp, projections, NULL, 2,
                            sfm_project_point3_mot, NULL, 
                            (void *) (&global_params),
                            NumIteration, VERBOSITY, opts, info,
                            use_constraints, constraints);

    }
    
    printf("[run_sfm] Number of iterations: %d\n", (int) info[5]);
    printf("info[6] = %0.3f\n", info[6]);
    printf("info[0] = %0.3f\n", info[0]);
    /* Copy out the params */
    for (j = 0; j < num_cameras; j++) 
    {
	double *dt = params + cnp * j + 0;
	double *w = params + cnp * j + 3;
	double Rnew[9];
    int c;

	/* Translation */
	init_camera_params[j].t[0] = dt[0];
    init_camera_params[j].t[1] = dt[1];
    init_camera_params[j].t[2] = dt[2];
	
	/* Rotation */
	rot_update(init_camera_params[j].R, w, Rnew);

        
    //matrix_print(3, 3, Rnew);   
	memcpy(init_camera_params[j].R, Rnew, 9 * sizeof(double));
        
	/* Focal length */
	if (est_focal_length) 
    {
        c = 7;
	    init_camera_params[j].f = 
                params[cnp * j + 6] / init_camera_params[j].f_scale;
     } 
    else 
    {
         c = 6;
     }
        
     printf("focal_length %f \n", init_camera_params[j].f );  
    }

    base = num_camera_params;
    for (i = 0; i < num_pts; i++) 
    {
	Vx(init_pts[i]) = params[base + 3 * i + 0];
	Vy(init_pts[i]) = params[base + 3 * i + 1];
	Vz(init_pts[i]) = params[base + 3 * i + 2];
   // printf("V3D %f %f %f \n", Vx(init_pts[i]),Vy(init_pts[i]),Vz(init_pts[i]) );  
    }
  free(params);
}
