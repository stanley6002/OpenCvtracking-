
#include "epipolar.h"

#define  CLAMP(x,mn,mx) (((x) < mn) ? mn : (((x) > mx) ? mx : (x))


int EstimatePose5Point(v2_t* k1, 
                       v2_t* k2, 
                       int matches, 
                       int num_trials, double threshold, 
                       double *K1, double *K2, 
                       double *R, double *t)
{
    int num_pts = (int) matches;
  //  v2_t *k1_pts = new v2_t[10];
  //  v2_t *k2_pts = new v2_t[10];
    
  //  for (int i = 0; i < num_pts; i++) {
  //      int idx1 = matches[i].m_idx1;
  //      int idx2 = matches[i].m_idx2;
        
  //      k1_pts[i] = v2_new(k1[idx1].m_x, k1[idx1].m_y);
  //      k2_pts[i] = v2_new(k2[idx2].m_x, k2[idx2].m_y);
  // }
    
  int num_inliers = compute_pose_ransac(num_pts, k1, k2, 
                                        K1, K2, threshold, num_trials, R, t);
    
   
//    delete [] k1_pts;
//    delete [] k2_pts;
//    matrix_print(3,3, R);
//    for (int i=0;i<3;i++)
//    {
//       printf("%0.3f\n", t[i]);
//    }
    return num_inliers;
}

int estimate_fmatrix_ransac_matches(int num_pts, v3_t *a_pts, v3_t *b_pts, 
                                     int num_trials, double threshold, 
                                     double success_ratio,
                                     int essential, double *F) 
{


     int i, j, k, idx;
     
     v3_t l_pts_best[8], r_pts_best[8];
     
     double Fbest[9];
     double *resid;
     double error_min;
     int inliers_max;
     
     double *a_matrix, *b_matrix;
     
     // double threshold = 1.0e-10;
     
     // srand(time(0));
     
    /* Make an array of all good correspondences */
   if (num_pts < 8) 
    {
         printf("[estimate_fmatrix_ransac] Could not find 8 good correspondences,"
               "F-matrix estimation failed\n");
         return 0;
     }
 
    
//     a_matrix = (double *)malloc(sizeof(double) * 3 * num_pts);
//     b_matrix = (double *)malloc(sizeof(double) * 3 * num_pts);
//     for (i = 0; i < num_pts; i++) 
//     {
//         a_matrix[i] = Vx(a_pts[i]);
//         a_matrix[i+num_pts] = Vy(a_pts[i]);
//         a_matrix[i+2*num_pts] = Vz(a_pts[i]);
//         
//         b_matrix[i] = Vx(b_pts[i]);
//         b_matrix[i+num_pts] = Vy(b_pts[i]);
//         b_matrix[i+2*num_pts] = Vz(b_pts[i]);        
//     }

    
     error_min = DBL_MAX;
     inliers_max = 0;
     resid = (double *) malloc(sizeof(double) * num_pts);
     //srand(time(0));
         /* Estimate the F-matrix using RANSAC */
         for (i = 0; i < num_trials; i++) {
         int idxs[8];
         v3_t l_pts[8], r_pts[8];
         double Ftmp[9], e1_tmp[3], e2_tmp[3];
         // double error;
         int num_inliers = 0;
         int success, nan = 0;
         int round = 0;
       
         /* Sample 8 random correspondences */
         for (j = 0; j < 8; j++) {
             int reselect = 0;
             
             if (round == 10000)   ///
                 return 0;
                idx = rand() % num_pts;             
         /* Make sure we didn't sample this index yet */
             for (k = 0; k < j; k++)
             {
                 if (idx == idxs[k] ||
                     (Vx(a_pts[idx]) == Vx(a_pts[idxs[k]]) &&
                      Vy(a_pts[idx]) == Vy(a_pts[idxs[k]]) &&
                      Vz(a_pts[idx]) == Vz(a_pts[idxs[k]])) ||
                     (Vx(b_pts[idx]) == Vx(b_pts[idxs[k]]) &&
                      Vy(b_pts[idx]) == Vy(b_pts[idxs[k]]) &&
                      Vz(b_pts[idx]) == Vz(b_pts[idxs[k]]))) 
                 {
                         reselect = 1;
                         break;
                     }
             }
             
             if (reselect) {
                 round++;
                 j--;
                 continue;
             }
             
             idxs[j] = idx;
         }
         /* Fill in the left and right points */
         for (j = 0; j < 8; j++) 
         {
             l_pts[j] = b_pts[idxs[j]];
             r_pts[j] = a_pts[idxs[j]];
         }
         /* Estimate the F-matrix */
         success = estimate_fmatrix_linear(8, r_pts, l_pts, essential, 
                                           Ftmp, e1_tmp, e2_tmp);
        
         if (success == 0)
             nan = 1;
         
         for (j = 0; j < 9; j++) {
             if (Ftmp[j] != Ftmp[j] /* isnan(Ftmp[j]) */) {
                 printf("[estimate_fmatrix_ransac_matches] nan encountered\n");
                 nan = 1;
                 break;
             }
         }
        
         /* Check for nan entries */
         if (isnan(Ftmp[0]) || isnan(Ftmp[1]) || isnan(Ftmp[2]) ||
             isnan(Ftmp[3]) || isnan(Ftmp[4]) || isnan(Ftmp[5]) ||
             isnan(Ftmp[6]) || isnan(Ftmp[7]) || isnan(Ftmp[8])) 
         {
             printf("[estimate_fmatrix_ransac_matches] "
                    "nan matrix encountered\n");
             nan = 1;
         }
         
         if (nan) {
             // error = DBL_MAX;
             num_inliers = 0;
         } else 
         {
             // printf("%0.3f\n", Ftmp[0]);
             
             /* Compute residuals */
    
 #if 1
             for (j = 0; j < num_pts; j++)
             {
                 resid[j] = fmatrix_compute_residual(Ftmp, a_pts[j], b_pts[j]);                      /////
               // resid[j] = fmatrix_compute_distance(Ftmp, a_pts[j], b_pts[j]);
                 if (resid[j] <threshold)      
                 num_inliers++;
             }
#endif
         
//             
 #if 0
//             /* Find the median */
//             error = median(num_pts, resid);
//             
//             if (error < error_min) {
//                 error_min = error;
//                 memcpy(Fbest, Ftmp, sizeof(double) * 9);
//                 memcpy(l_pts_best, l_pts, sizeof(v3_t) * 8);
//                 memcpy(r_pts_best, r_pts, sizeof(v3_t) * 8);
//             }
 #else
             if (num_inliers > inliers_max) 
             {
                 inliers_max = num_inliers;
                 memcpy(Fbest, Ftmp, sizeof(double) * 9);
                 memcpy(l_pts_best, l_pts, sizeof(v3_t) * 8);
                 memcpy(r_pts_best, r_pts, sizeof(v3_t) * 8);
             }
 #endif
            }
         }
//         
// #if 0
//         if (error < threshold)
//             break;
// #endif
//         
//         if ((double) num_inliers / num_pts > success_ratio)
//             break;
//     }
//     
//     // printf("Minimum error: %0.5e\n", error_min);
//     // printf("Maximum inliers: %d\n", inliers_max);
//     
//        matrix_print(3, 3, Fbest);
//     
      free(resid);
//     
//    /* Copy out the F-matrix */
      memcpy(F, Fbest, sizeof(double) * 9);   
    return inliers_max;
 }

int estimate_fmatrix_linear(int num_pts, v3_t *r_pts, v3_t *l_pts, 
                             int essential,
                             double *Fout, double *e1, double *e2) 
 {
     int i;
     v3_t r_c, l_c;
     double r_dist, l_dist, r_scale, l_scale;
     
     v3_t *r_pts_new, *l_pts_new;
     
     double *A, *b, X[8], F[9], H[9], H_p[9], tmp[9], F_new[9];
     double U[9], VT[9];
     
     v3_t r_pts_8pt[8], l_pts_8pt[8];
     double A_8pt[64], b_8pt[8];
     
     int success;
     
     /* Check that there are enough point correspondences */
     if (num_pts < 8) {
         printf("[estimate_fmatrix_linear] Insufficient correspondences "
                "(need at least 8, given only %d)\n", num_pts);
         return 0;
     }
     
     
     /* First, compute the centroid of both sets of points */
     
     r_c = v3_new(0.0, 0.0, 0.0);
     l_c = v3_new(0.0, 0.0, 0.0);
     
     for (i = 0; i < num_pts; i++)
     {
         r_c = v3_add(r_c, r_pts[i]);
         l_c = v3_add(l_c, l_pts[i]);
     }
     
         r_c = v3_scale(1.0 / num_pts, r_c);
         l_c = v3_scale(1.0 / num_pts, l_c);
     
     
     /* Compute the average distance from each point to the centroid */
         r_dist = l_dist = 0;
     
     for (i = 0; i < num_pts; i++) {
         r_dist += v3_mag(v3_sub(r_c, r_pts[i]));
         l_dist += v3_mag(v3_sub(l_c, l_pts[i]));
     }
     
     r_dist /= num_pts;
     l_dist /= num_pts;
     
     r_dist /= sqrt(2.0);
     l_dist /= sqrt(2.0);
     
     r_scale = 1.0 / r_dist;
     l_scale = 1.0 / l_dist;
     
     
     /* Normalize the points with an affine transform */
     if (num_pts > 8) 
     {
         r_pts_new = (v3_t *)malloc(sizeof(v3_t) * num_pts);
         l_pts_new = (v3_t *)malloc(sizeof(v3_t) * num_pts);
     } else 
     {
         r_pts_new = r_pts_8pt;
         l_pts_new = l_pts_8pt;
     }
     
     
     for (i = 0; i < num_pts; i++) {
         r_pts_new[i] = v3_scale(r_scale, v3_sub(r_pts[i], r_c));
         l_pts_new[i] = v3_scale(l_scale, v3_sub(l_pts[i], l_c));
         
         Vz(r_pts_new[i]) = 1.0;
         Vz(l_pts_new[i]) = 1.0;
     }
     
     
     /* Fill in the rows of the matrix A */
     if (num_pts > 8)
         A = (double *)malloc(sizeof(double) * 8 * num_pts);
     else
         A = A_8pt;
     
     for (i = 0; i < num_pts; i++) 
     {
         double u = Vx(l_pts_new[i]);
         double v = Vy(l_pts_new[i]);
         double u_p = Vx(r_pts_new[i]);
         double v_p = Vy(r_pts_new[i]);
         
         A[i * 8 + 0] = u * u_p;
         A[i * 8 + 1] = v * u_p;
         A[i * 8 + 2] = u_p;
         A[i * 8 + 3] = u * v_p;
         A[i * 8 + 4] = v * v_p;
         A[i * 8 + 5] = v_p;
         A[i * 8 + 6] = u;
         A[i * 8 + 7] = v;
     }
     
     
     /* Fill in the vector b */
     if (num_pts > 8)
         b = (double *)malloc(sizeof(double) * num_pts);
     else
         b = b_8pt;
     
     for (i = 0; i < num_pts; i++) {
         b[i] = -1.0;
     }
     
     
     /* Solve for the least-squares solution to the F-matrix */
     if (num_pts > 8)
         dgelsy_driver(A, b, X, num_pts, 8, 1);
     else
        // printf("A_matrix_print \n");
        // matrix_print(8, 8, A), 
         dgesv_driver(num_pts, A, b, X);
     
     /* Un-normalize */
     H[0] = l_scale;  H[1] =     0.0;  H[2] = -l_scale * Vx(l_c);
     H[3] =     0.0;  H[4] = l_scale;  H[5] = -l_scale * Vy(l_c);
     H[6] =     0.0;  H[7] =     0.0;  H[8] =                1.0;
     
     H_p[0] = r_scale;  H_p[3] =     0.0;  H_p[6] = -r_scale * Vx(r_c);
     H_p[1] =     0.0;  H_p[4] = r_scale;  H_p[7] = -r_scale * Vy(r_c);
     H_p[2] =     0.0;  H_p[5] =     0.0;  H_p[8] =                1.0;
     
     memcpy(F, X, sizeof(double) * 8);
     F[8] = 1.0;
     
     matrix_product(3, 3, 3, 3, H_p, F, tmp);
     matrix_product(3, 3, 3, 3, tmp, H, F_new);
     
     /* Use SVD to compute the nearest rank 2 matrix */
     if (essential == 0)
         success = closest_rank2_matrix(F_new, Fout, U, VT);
     else
         success = closest_rank2_matrix_ssv(F_new, Fout, U, VT);
     
     /* The last column of U spans the nullspace of F, so it is the
      * epipole in image A.  The last column of V spans the nullspace
      * of F^T, so is the epipole in image B */
     
     e1[0] = U[2];
     e1[1] = U[5];
     e1[2] = U[8];
     
     e2[0] = VT[6];
     e2[1] = VT[7];
     e2[2] = VT[8];
     
     
     /* Cleanup */
     if (num_pts > 8) {
         free(A);
         free(b);
         free(r_pts_new);
         free(l_pts_new);
     }
     
     return success;
 }

 int closest_rank2_matrix(double *Fin, double *Fout, double *U, double *VT) 
 {
     double S[3], sigma[9], F_rank2[9], tmp[9];
     
     int success = dgesvd_driver(3, 3, Fin, U, S, VT);
     // int retval = svd3_driver(Fin, U, S, VT);
     
     sigma[0] = S[0];  sigma[1] =  0.0;  sigma[2] =  0.0;
     sigma[3] =  0.0;  sigma[4] = S[1];  sigma[5] =  0.0;
     sigma[6] =  0.0;  sigma[7] =  0.0;  sigma[8] =  0.0;
     
     matrix_product(3, 3, 3, 3, U, sigma, tmp);
     matrix_product(3, 3, 3, 3, tmp, VT, F_rank2);
     
     memcpy(Fout, F_rank2, sizeof(double) * 9);
     
     return success;
     // return (retval == 0);
 }
// 
int closest_rank2_matrix_ssv(double *Fin, double *Fout, 
                              double *U, double *VT)
 {
     double S[3], sigma[9], F_rank2[9], tmp[9];
     
     // int success = dgesvd_driver(3, 3, Fin, U, S, VT);
     int retval = svd3_driver(Fin, U, S, VT);
     
     sigma[0] =  1.0;  sigma[1] =  0.0;  sigma[2] =  0.0;
     sigma[3] =  0.0;  sigma[4] =  1.0;  sigma[5] =  0.0;
     sigma[6] =  0.0;  sigma[7] =  0.0;  sigma[8] =  0.0;
     
     matrix_product(3, 3, 3, 3, U, sigma, tmp);
     matrix_product(3, 3, 3, 3, tmp, VT, F_rank2);
     
     memcpy(Fout, F_rank2, sizeof(double) * 9);
     
     // return success;
     return (retval == 0);
 }
// 
 int svd3_driver(double *A, double *U, double *S, double *VT) 
 {
     double V[9], Utmp[9], VTtmp[9], UT[9];
     int retval = svd(3, 3, 1, 1, 1.0e-4, 1.0e-4, A, S, Utmp, V, VTtmp);
     int perm[3];
     
     if (retval != 0)
         return retval;
     
     qsort_descending();
     qsort_perm(3, S, perm);
     
     matrix_transpose(3, 3, Utmp, UT);
     memcpy(Utmp + 0, UT + perm[0] * 3, 3 * sizeof(double));
     memcpy(Utmp + 3, UT + perm[1] * 3, 3 * sizeof(double));
     memcpy(Utmp + 6, UT + perm[2] * 3, 3 * sizeof(double));
     matrix_transpose(3, 3, Utmp, U);
     
     memcpy(VT + 0, VTtmp + perm[0] * 3, 3 * sizeof(double));
     memcpy(VT + 3, VTtmp + perm[1] * 3, 3 * sizeof(double));
     memcpy(VT + 6, VTtmp + perm[2] * 3, 3 * sizeof(double));
     
     return retval;
 }
// 
double fmatrix_compute_residual(double *F, v3_t r, v3_t l) 
 {
     double Fl[3], Fr[3], pt; 
     double error;
     
// #if 1
     Fl[0] = F[0] * Vx(l) + F[1] * Vy(l) + F[2] * Vz(l);
     Fl[1] = F[3] * Vx(l) + F[4] * Vy(l) + F[5] * Vz(l);
     Fl[2] = F[6] * Vx(l) + F[7] * Vy(l) + F[8] * Vz(l);
     
     Fr[0] = F[0] * Vx(r) + F[3] * Vy(r) + F[6] * Vz(r);
     Fr[1] = F[1] * Vx(r) + F[4] * Vy(r) + F[7] * Vz(r);
     Fr[2] = F[2] * Vx(r) + F[5] * Vy(r) + F[8] * Vz(r);
     
     pt = Vx(r) * Fl[0] + Vy(r) * Fl[1] + Vz(r) * Fl[2];
//#else
//    matrix_product(3, 3, 3, 1, F, l.p, Fl);
//    matrix_transpose_product(3, 3, 3, 1, F, r.p, Fr);
     
//    matrix_product(1, 3, 3, 1, r.p, Fl, &error);
//#endif
//    printf("Residuals: %0.5f\n", sqrt(pt*pt));


//     std::cout<<"error2 : "<< (1.0 / (Fl[0] * Fl[0] + Fl[1] * Fl[1]) +
//                           1.0 / (Fr[0] * Fr[0] + Fr[1] * Fr[1])) *
//                          (pt * pt)<<"\n";
//     return(sqrt(pt*pt));
   return ((1.0 / (Fl[0] * Fl[0] + Fl[1] * Fl[1]) +
 	 1.0 / (Fr[0] * Fr[0] + Fr[1] * Fr[1])) *
 	(pt * pt));
 }
double fmatrix_compute_distance(double *F, v3_t r, v3_t l) 
{
    double Fl[3], Fr[3], pt; 
    double error;
    
    // #if 1
    Fl[0] = F[0] * Vx(l) + F[1] * Vy(l) + F[2] * Vz(l);
    Fl[1] = F[3] * Vx(l) + F[4] * Vy(l) + F[5] * Vz(l);
    Fl[2] = F[6] * Vx(l) + F[7] * Vy(l) + F[8] * Vz(l);
    
    Fr[0] = F[0] * Vx(r) + F[3] * Vy(r) + F[6] * Vz(r);
    Fr[1] = F[1] * Vx(r) + F[4] * Vy(r) + F[7] * Vz(r);
    Fr[2] = F[2] * Vx(r) + F[5] * Vy(r) + F[8] * Vz(r);
    
    pt = Vx(r) * Fl[0] + Vy(r) * Fl[1] + Vz(r) * Fl[2];
    //#else
    //    matrix_product(3, 3, 3, 1, F, l.p, Fl);
    //    matrix_transpose_product(3, 3, 3, 1, F, r.p, Fr);
    
    //    matrix_product(1, 3, 3, 1, r.p, Fl, &error);
    //#endif
    //    printf("Residuals: %0.5f\n", sqrt(pt*pt));
    
    
    //     std::cout<<"error2 : "<< (1.0 / (Fl[0] * Fl[0] + Fl[1] * Fl[1]) +
    //                           1.0 / (Fr[0] * Fr[0] + Fr[1] * Fr[1])) *
    //                          (pt * pt)<<"\n";
    return(sqrt(pt*pt));
    //return ((1.0 / (Fl[0] * Fl[0] + Fl[1] * Fl[1]) +
    //         1.0 / (Fr[0] * Fr[0] + Fr[1] * Fr[1])) *
    //        (pt * pt));
}

static v3_t *global_ins = NULL;
static v3_t *global_outs = NULL;
static int global_num_matches = 0;
static double global_scale;

void fmatrix_residuals(int *m, int *n, double *x, double *fvec, int *iflag)
{
    int i;
    double sum = 0.0;
    
    double F[9], F2[9], U[9], VT[9];
    memcpy(F, x, sizeof(double) * 8);
    F[8] = global_scale;
    
    closest_rank2_matrix(F, F2, U, VT);
    
    if (global_num_matches != (*m)) 
    {
        printf("Error: number of matches don't match!\n");
    }
    
    for (i = 0; i < global_num_matches; i++)
    {
        fvec[i] = ((fmatrix_compute_residual(F2, global_outs[i], global_ins[i])));
        if (*iflag == 0) 
        {
            sum += fvec[i];
        }
    }
    
    #if 0
    if (*iflag == 0) {
        matrix_print(3, 3, F);
        matrix_print(3, 3, F2);
        printf("Residuals: %0.5f\n", sum);
    }
   #endif
}
void refine_fmatrix_nonlinear_matches(int num_pts, v3_t *r_pts, v3_t *l_pts, 
                                      double *F0, double *Fout)
{
    double Ftmp[9];
    double U[9], VT[9];
    
    global_ins = l_pts;
    global_outs = r_pts;
    global_num_matches = num_pts;
    global_scale = F0[8];    
    memcpy(Ftmp, F0, sizeof(double) * 9);
    int x=8;
    
    lmdif_driver2(fmatrix_residuals, num_pts, x, Ftmp, 0.00001);   
    Ftmp[8] = global_scale;
    //matrix_print(3, 3, Ftmp);
    closest_rank2_matrix(Ftmp, Fout, U, VT);
    //matrix_print(3, 3, Fout);    
    global_ins = global_outs = NULL;
    global_num_matches = 0;    
}
v3_t Triangulate(v2_t p, v2_t q, double *camera_1R,double *camera_1t,double *camera_2R, double *camera_2t,  
                 double proj_error, bool in_front, double angle,
                 bool explicit_camera_centers, double *K1, double *K2)
{
    
    double K1inv[9], K2inv[9];
    
    
    matrix_invert(3, K1, K1inv);
    matrix_invert(3, K2, K2inv);
    
    /* Set up the 3D point */
    //    // EDIT!!!
    double proj1[3] = { Vx(p), Vy(p), -1.0 };
    double proj2[3] = { Vx(q), Vy(q), -1.0 };
    
    double proj1_norm[3], proj2_norm[3];
    //    
    matrix_product(3, 3, 3, 1, K1inv, proj1, proj1_norm);
    matrix_product(3, 3, 3, 1, K2inv, proj2, proj2_norm);
    
    v2_t p_norm = v2_new(proj1_norm[0] / proj1_norm[2],
                         proj1_norm[1] / proj1_norm[2]);
    
    v2_t q_norm = v2_new(proj2_norm[0] / proj2_norm[2],
                         proj2_norm[1] / proj2_norm[2]);
    
    //v2_t p_norm = v2_new(-proj1_norm[0], -proj1_norm[1] );
    //v2_t q_norm = v2_new(-proj2_norm[0], -proj2_norm[1] );
    ////    
    ////    /* Undo radial distortion */
    //    p_norm = UndistortNormalizedPoint(p_norm);
    //    q_norm = UndistortNormalizedPoint(q_norm);
    //    
    //    /* Compute the angle between the rays */
    
    //    
    /* Triangulate the point */
    v3_t pt;
    //    if (!explicit_camera_centers) {
    //        pt = triangulate(p_norm, q_norm, c1.R, c1.t, c2.R, c2.t, &proj_error);
    //    } else {
    double t1[3];
    double t2[3];
     //     angle = ComputeRayAngle(p, q,K1inv,K2inv,camera_1R,camera_2R,camera_1t,camera_2t);
    //    printf("angle: %f \n", angle);
    //      /* Put the translation in standard form */
    
    matrix_product(3, 3, 3, 1, camera_1R, camera_1t, t1);
    matrix_scale(3, 1, t1, -1.0, t1);
    matrix_product(3, 3, 3, 1,  camera_2R, camera_2t, t2);
    matrix_scale(3, 1, t2, -1.0, t2);
   
    //matrix_print(3, 3, camera_1R);
    //matrix_print(3, 3, camera_2R);
    //matrix_print(3, 1, camera_1t);
    //matrix_print(3, 1, camera_2t);

    pt = triangulate(p_norm, q_norm, camera_1R,t1,camera_2R,t2, &proj_error);
    
    
//    double qp[3];
//    double x[3];
//    x[0]=pt.p[0];
//    x[1]=pt.p[1];
//    x[2]=pt.p[2];
//    matrix_product331(camera_2R, x, qp);
//	  qp[0] += t1[0];
//	  qp[1] += t1[1];
//	  qp[2] += t1[2];	
//    
    
    
    
    //      proj_error = (530 + 530) * 0.5 * sqrt(proj_error * 0.5);
    //      printf ("Proj_error: %f \n", proj_error);
    //    
    //    /* Check cheirality */
    //      bool cc1 = CheckCheirality(pt,camera_1R,camera_1t);
    //      bool cc2 = CheckCheirality(pt,camera_2R,camera_2t);
    //      in_front = (cc1 && cc2);
    return pt;
}

double ComputeRayAngle(v2_t p, v2_t q, double *K1_inv, double *K2_inv, double*camera_1R, double *camera_2R,double *camera_1t, double *camera_2t)
{
    
    double p3[3] = { Vx(p), Vy(p), 1.0 };
    double q3[3] = { Vx(q), Vy(q), 1.0 };
    
    double p3_norm[3], q3_norm[3];
    matrix_product331(K1_inv, p3, p3_norm);
    matrix_product331(K2_inv, q3, q3_norm);
    
    v2_t p_norm = v2_new(p3_norm[0] / p3_norm[2], p3_norm[1] / p3_norm[2]);
    v2_t q_norm = v2_new(q3_norm[0] / q3_norm[2], q3_norm[1] / q3_norm[2]);
    
    double R1_inv[9], R2_inv[9];
    matrix_transpose(3, 3, (double *) camera_1R, R1_inv);
    matrix_transpose(3, 3, (double *) camera_2R, R2_inv);
    
    double p_w[3], q_w[3];
    
    double pv[3] = { Vx(p_norm), Vy(p_norm), -1.0 };
    double qv[3] = { Vx(q_norm), Vy(q_norm), -1.0 };
    
    double Rpv[3], Rqv[3];
    
    matrix_product331(R1_inv, pv, Rpv);
    matrix_product331(R2_inv, qv, Rqv);
    
    matrix_sum(3, 1, 3, 1, Rpv, (double *) camera_1t, p_w);
    matrix_sum(3, 1, 3, 1, Rqv, (double *) camera_2t, q_w);
    
    /* Subtract out the camera center */
    double p_vec[3], q_vec[3];
    matrix_diff(3, 1, 3, 1, p_w, (double *) camera_1t, p_vec);
    matrix_diff(3, 1, 3, 1, q_w, (double *) camera_2t, q_vec);
    
    /* Compute the angle between the rays */
    double dot;
    matrix_product(1, 3, 3, 1, p_vec, q_vec, &dot);
    
    double mag = matrix_norm(3, 1, p_vec) * matrix_norm(3, 1, q_vec);
    
    double angle= (dot / mag);
    
    return (angle);
}

//bool CheckCheirality(v3_t p, double * camera_R, double *camera_t) 
//{
//    double pt[3] = { Vx(p), Vy(p), Vz(p) };
//    double cam[3];
//    
//    pt[0] -= camera_t[0];
//    pt[1] -= camera_t[1];
//    pt[2] -= camera_t[2];
//    matrix_product(3, 3, 3, 1, (double *) camera_R, pt, cam);
//    
//    // EDIT!!!
//    if (cam[2] > 0.0)
//        return false;
//    else
//        return true;
//}
//v2_t UndistortNormalizedPoint(v2_t p) 
//{
//    int POLY_INVERSE_DEGREE=6;
//    double k_inv[6];
//    k_inv[0]=k_inv[2]=k_inv[3]=0.0; 
//    k_inv[4]=k_inv[5]=0.0; 
//    k_inv[1] = 1.0;
//    double r = sqrt(Vx(p) * Vx(p) + Vy(p) * Vy(p));
//    if (r == 0.0)
//        return p;
//    
//    double t = 1.0;
//    double a = 0.0;
//    
//    for (int i = 0; i < POLY_INVERSE_DEGREE; i++) {
//        a += t * k_inv[i];
//        t = t * r;
//    }
//    
//    double factor = a / r;
//    
//    return v2_scale(factor, p);
//}


float  Apical_Angle( v2_t* l_pt, v2_t* r_pt, double *R,  double *t, double *K , const int size_)
{
     int i;
     int j =0;
     double K1_inv[9];
     double camera2t[3];
     double t_scale[3]; 
     
     # define Bin_Number 200
    
      //printf("Rotation R from 5pts : \n");
      //matrix_print(3, 3, R);
      //printf("Translation T from 5pts: \n");
      //matrix_print(3,1,t);
    
     matrix_invert(3, K, K1_inv);
    
     matrix_transpose_product(3, 3, 3, 1, R, t, camera2t);
    
     matrix_scale(3, 1, camera2t, -1.0, t_scale);
    
     // matrix_print(3,3,R);
     // matrix_print(3,1,t);
     // printf(" t_distance %f\n", sqrt((t[0]*t[0])+(t[1]*t[1])+(t[2]*t[2])));
    
    double * Tao_temp;
    
    Tao_temp =(double *) malloc(size_*sizeof(double));

    double Tao_val;
    
    for (i = 0; i < size_ ; i++)
    {
        double x_temp[3];
        double r[3] = { Vx(r_pt[i]), Vy(r_pt[i]), 1.0 };
        double l[3] = { Vx(l_pt[i]), Vy(l_pt[i]), 1.0 };
        double r_norm[3], l_norm[3];
        
        matrix_product331(K1_inv, r, r_norm);
        matrix_product331(K1_inv, l, l_norm);
        matrix_product331(R, r_norm, x_temp);
        
         double A[6];
         double B[3];
         double Result[2];
        
         A[0]= l_norm[0],A[1]= -x_temp[0];
         A[2]= l_norm[1],A[3]= -x_temp[1];
         A[4]= l_norm[2],A[5]= -x_temp[2];
    
        
         B[0]= -t_scale[0], B[1]= -t_scale[1], B[2]= -t_scale[2];
      
         dgelsy_driver(A, B, Result , 3, 2, 1);
         
        //printf("result \n");
        //matrix_print(2,1,Result);
        
    
        Tao_val= acos(1-(1/(2*(Result[1]*Result[1]))));
        Tao_temp[i]= Tao_val;

        if (checkna(Tao_temp[i])==true)
        {
            j+=1;
        }

    }   
    //matrix_print(size_,1,Tao_temp);

    double* Tao = (double *) malloc((size_-j)*sizeof(double)); 
    
    // reinitialized parameters 
    
        i=0;
        j=0;
    
    for (i=0;i< size_;i++)
    {
       if(checkna(Tao_temp[i]) == false)
       {
           Tao[j]= Tao_temp[i];
           j+=1;
       }
    }
       
     
     //matrix_print(j,1,Tao);
     float angle =  KDE_estimation(Tao, j , Bin_Number);
     free(Tao_temp);
     free(Tao);
     return(angle);
}

float KDE_estimation(double * Tao, const int size_, const int Nbins)
{

  //double max_number = -9999;
  double max_number =  0.4;    // remove outliers from candidated points 
  double min_number =  0.0;
  double range;
  int i;
  //int * Bin;
  //Bin =(int *) malloc(Nbins* sizeof(int));
  //memset( Bin, 0, sizeof(Bin));
  int Bin[200]={};
  //int index   = 0; 
  int mx_bin  = 0;
  int mx_index= 0;
  double  Range_low;
  double  Range_upper;
  //double  Apical_Angle;
  //bool *Index_vector;
    
    int numofmin=0;  
  
   // for (i=0;i< size_;i++)
   // {

   //     if (Tao[i]<min_number)
   //     {
   //      min_number= Tao[i];
   //     }
       //if (Tao[i]> max_number)
       // {
       //  max_number= Tao[i];
       // }
    //}


 
   range = (max_number - min_number) / Nbins;
   
    for(int index=0; index< Nbins; index++)
    {
        for(i=0; i<size_;i++)
        {
            Range_low = min_number + (index)*range;
            Range_upper = min_number + (index+1)*range;
               if ( Range_low <Tao[i] && Tao[i]<= Range_upper)              
               {
                    Bin[index] += 1;
                   
               }
        }

        if(Bin[index]>mx_bin)
        {
           mx_bin = Bin[index];
           mx_index = index;

        }


       //printf (" range_low:  %f \n",(min_number + (index)*range) );
       //printf (" range_upper :  %f \n",(min_number + (index+1)*range) );

    }
    
    //for(i=0; i<size_;i++)
    //{
    //  if ((Tao[i] >= min_number) && (Tao[i] > min_number+0.05 ))
    //      numofmin++;
    //}
    
    //printf("index %i \n", mx_index);
    //printf("mx_bin %i \n", mx_bin);
    // printf("mx_index range min max numofmax %d %f %f %f %d\n",mx_index, range, min_number, max_number, numofmin);
    // float Apical_angle = (((min_number + (mx_index+1)*range)+(min_number + (mx_index)*range)))*0.5;
    float Apical_angle = (min_number+(mx_index)*range);
     //    printf("value %f \n ", Apical_angle);
     //    for(i=0;i<Nbins;i++)
     //    {
     //      printf("Bin_ %i \n", Bin[i]);
     //    }
    return(Apical_angle);
    
}
 bool  checkna (double tao)
{
 
   return (tao != tao);

}
