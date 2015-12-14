//
//  main.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 7/1/14.
//  Copyright 2014 __MyCompanyName__. All rights reserved.
//

#include "F_matrix.h"
#include <iostream>
#include <vector>

using namespace std;

void F_matrix_process (int num_pts, v3_t* r_pt, v3_t* l_pt, double *F_final, int num_trial, int F_threshold, int essential, matched* refined_pts, int ith_pair, float thres)
{
    int F_estimated;
    double F[9];
    F_estimated= estimate_fmatrix_ransac_matches(num_pts, r_pt, l_pt, num_trial, 5.0, 2.0,essential, F);
   
    vector<int> inliers;
    
    for (int i=0; i<num_pts;i++)
    {
        double distance = fmatrix_compute_residual(F,r_pt[i],l_pt[i]);
        if (distance<thres)
        { 
        //cout<<distance<<endl;
        inliers.push_back(i);
        }
    }
    
    cout<<"feature_point_Before"<<num_pts<<" "<< "feature_point_screened by f_matrix "<<(int)inliers.size()<<endl;
    //cout<< error_brefine/(int)inliers.size()<<endl;
    v3_t *l_ptinlier= new v3_t[(int)inliers.size()];
    v3_t *r_ptinlier= new v3_t[(int)inliers.size()];

    for (int i=0; i<(int)inliers.size();i++)
    {
        //CvPoint newmatched;  
        l_ptinlier[i]= l_pt[inliers[i]];
        r_ptinlier[i]= r_pt[inliers[i]];
       
    }
 
    double F0[9];
    memcpy(F0,F,sizeof(double)*9);
    if (!essential)
    {
        for (int i=0; i<(int)inliers.size();i++)
        {
            //CvPoint newmatched;  
            l_ptinlier[i]= l_pt[inliers[i]];
            r_ptinlier[i]= r_pt[inliers[i]];
        }
       refine_fmatrix_nonlinear_matches((int)inliers.size(), r_ptinlier, l_ptinlier, F0, F); 
    }
    
    vector<int> non_inliers;
    for (int i=0; i<num_pts;i++)
    {
        double distance = fmatrix_compute_residual(F,r_pt[i],l_pt[i]);
        // double distance= fmatrix_compute_distance(F,  r_pt[i],  l_pt[i]); 
      
        if (distance<thres)
        {
            non_inliers.push_back(i);
          }
    }
    v3_t *l_ptinlier_nonlinear= new v3_t[(int)non_inliers.size()];
    v3_t *r_ptinlier_nonlinear= new v3_t[(int)non_inliers.size()];
    for (int i=0; i<(int)non_inliers.size();i++)
    {
        l_ptinlier_nonlinear[i]= l_pt[non_inliers[i]];
        r_ptinlier_nonlinear[i]= r_pt[non_inliers[i]];
    }

   
    push_backpts(l_ptinlier_nonlinear, r_ptinlier_nonlinear, refined_pts,(int)non_inliers.size(),ith_pair);
    double error;
    double sum=0;
    for (int i=0;i<(int)inliers.size();i++)
    {
        error=fmatrix_compute_residual(F, r_ptinlier[i],l_ptinlier[i]) ;
        sum+=error;
    }
    
    memcpy(F_final,F,9*sizeof(double));
    
    delete []  l_ptinlier_nonlinear;
    delete []  r_ptinlier_nonlinear;
    delete []  l_ptinlier;
    delete []  r_ptinlier;
        
}


void push_backpts(v3_t *lpts, v3_t*rpts, matched *pts, int inlier_size, int ith_pair)
{
    for (int i=0;i<inlier_size;i++)
    {
  
        pts[ith_pair].L_pts.push_back(lpts[i]);
        pts[ith_pair].R_pts.push_back(rpts[i]);
    }
}


void pop_backpts_WI(v2_t*lpts, v2_t*rpts, matched *pts, int ith_pair)
{
    for (int i=0; i<(int)pts[0].L_pts.size();i++)
    {
        lpts[i].p[0]= pts[ith_pair].L_pts[i].p[0];
        lpts[i].p[1]= pts[ith_pair].L_pts[i].p[1];       
        rpts[i].p[0]= pts[ith_pair].R_pts[i].p[0];
        rpts[i].p[1]= pts[ith_pair].R_pts[i].p[1];
    }
}

void pop_backpts(v3_t*lpts, v3_t*rpts, matched *pts)
{
    for (int i=0; i<(int)pts[0].L_pts.size();i++)
    {
        lpts[i]= pts[0].L_pts[i];
        rpts[i]= pts[0].R_pts[i];
    }
}

void pushback_Fmatrix(double* F, F_key_matrix *F_matrix,int i)
{
    F_matrix[i].p[0]=F[0];
    F_matrix[i].p[1]=F[1];
    F_matrix[i].p[2]=F[2];
    F_matrix[i].p[3]=F[3];
    F_matrix[i].p[4]=F[4];
    F_matrix[i].p[5]=F[5];
    F_matrix[i].p[6]=F[6];
    F_matrix[i].p[7]=F[7];
    F_matrix[i].p[8]=F[8];
}
void  EstimateTransform(v2_t*lpts, v2_t*rpts, Motion MotionSelect ,int num_size,  int _round /*128 m_homography_rounds*/, 
                         int _homography_threshold/*6.0m_homography_threshold*/,double *H, double *K)
{
    int min_matches;
    switch (MotionSelect) 
    {
        case MotionRigid:
            min_matches = 3;
            break;
        case MotionHomography:
            min_matches = 4;
            break;
    }

    int *match_idxs = new int[min_matches];
    int num_matches = num_size;
    int max_inliers = 0;
    double Mbest[9];
    
    if (num_matches < min_matches) {
        std::vector<int> empty;
        printf("Cannot estimate rigid transform\n");
        //return empty;
    }
    
    v3_t *r_pts_ran = new v3_t[min_matches];
    v3_t *l_pts_ran = new v3_t[min_matches];
    double *weight = new double[min_matches];
    for (int round = 0; round < 64 ; round++) 
    {
        for (int i = 0; i < min_matches; i++) {
            bool found;
            int idx;
            
            do {
                found = true;
                idx = rand() % num_matches;
                
                for (int j = 0; j < i; j++) {
                    if (match_idxs[j] == idx) {
                        found = false;
                        break;
                    }
                }
            } while (!found);
            
            match_idxs[i] = idx;
        }
     
        /* Solve for the motion */
		
        for (int i = 0; i < min_matches; i++) 
        {
           int index = match_idxs[i];
            
            l_pts_ran[i].p[0] = lpts[index].p[0];
            l_pts_ran[i].p[1] = lpts[index].p[1];
            l_pts_ran[i].p[2] = 1.0;
		    
            r_pts_ran[i].p[0] = rpts[index].p[0];
            r_pts_ran[i].p[1] = rpts[index].p[1];
            r_pts_ran[i].p[2] = 1.0;
            
            weight[i] = 1.0;
        }
    
                
        double M_current[9];
    
        
        switch (MotionSelect) {
            case MotionRigid:
            {
                double R[9], T[9], Tout[9], scale;
                align_horn(min_matches, r_pts_ran, l_pts_ran, R, T, Tout, &scale, weight);
                memcpy(M_current , Tout, 9 * sizeof(double));
                break;
            }
                
            case MotionHomography: 
            {
               //align_homography(min_matches, r_pts, l_pts, Mcurr, 0);
               // break;
            }
        }
		
        
        std::vector<int> inliers;
        int num_inliers = CountInliers(lpts, rpts, M_current, 
                                       _homography_threshold, inliers, num_size);
        //cout<<"# inliers " <<num_inliers <<endl;
        if (num_inliers > max_inliers) 
        {
            max_inliers = num_inliers;
            memcpy(Mbest, M_current, 9 * sizeof(double));
        }
    }
    matrix_print(3,3,Mbest);


}

static int CountInliers(const v2_t* lpts, 
                        const v2_t* rpts, 
                        double *M, double thresh, std::vector<int> &inliers, int num_size)
{
    inliers.clear();
    int count = 0;
    
    for (unsigned int i = 0; i < num_size; i++) 
    {
        /* Determine if the ith feature in f1, when transformed by M,
         * is within RANSACthresh of its match in f2 (if one exists)
         *
         * if so, increment count and append i to inliers */
       
        double p[3];
        
        p[0] = lpts[i].p[0];
        p[1] = lpts[i].p[1];
        p[2] = 1.0;
        
        double q[3];
        matrix_product(3, 3, 3, 1, M, p, q);
        
        double qx = q[0] / q[2];
        double qy = q[1] / q[2];
        
        double dx = qx - rpts[i].p[0];
        double dy = qy - rpts[i].p[1];
        
        double dist = sqrt(dx * dx + dy * dy);
      
        if (dist <= thresh) 
        {
            count++;
            inliers.push_back(i);
        }
    }
    
    return count;
    
}
double align_horn(int n, v3_t *right_pts, v3_t *left_pts, 
                  double *R, double *T, 
                  double *Tout, double *scale, double *weight) {
    int i;
    v3_t right_centroid = v3_new(0.0, 0.0, 0.0);
    v3_t left_centroid = v3_new(0.0, 0.0, 0.0);
    double M[2][2] = { { 0.0, 0.0 }, 
        { 0.0, 0.0 } };
    double MT[2][2];
    double MTM[2][2];
    double eval[2], sqrteval[2];
    double evec[2][2];
    double S[2][2], Sinv[2][2], U[2][2];
    double Tcenter[3][3] = { { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 } };
    
    double Ttmp[3][3];
    
    double sum_num, sum_den, RMS_sum;
    
#if 1
    double weight_sum = 0.0;
    
    if (weight == NULL) {
        weight_sum = n;
        
        for (i = 0; i < n; i++) {
            right_centroid = 
            v3_add(right_centroid, right_pts[i]);
            left_centroid = 
            v3_add(left_centroid, left_pts[i]);
        }
        
        right_centroid = v3_scale(1.0 / weight_sum, right_centroid);
        left_centroid = v3_scale(1.0 / weight_sum, left_centroid);        
    } else {
        /* Compute the weighted centroid of both point sets */
        for (i = 0; i < n; i++) {
            right_centroid = 
            v3_add(right_centroid, v3_scale(weight[i], right_pts[i]));
            left_centroid = 
            v3_add(left_centroid, v3_scale(weight[i], left_pts[i]));
            weight_sum += weight[i];
            
        }
        
        right_centroid = v3_scale(1.0 / weight_sum, right_centroid);
        left_centroid  = v3_scale(1.0 / weight_sum, left_centroid);
    
    }
#else
    /* Calculate the centroid of both sets of points */
    for (i = 0; i < n; i++) {
        right_centroid = v3_add(right_centroid, right_pts[i]);
        left_centroid = v3_add(left_centroid, left_pts[i]);
    }
    
    right_centroid = v3_scale(1.0 / n, right_centroid);
    left_centroid = v3_scale(1.0 / n, left_centroid);
#endif
    
    /* Compute the scale */
    sum_num = sum_den = 0.0;
    
    for (i = 0; i < n; i++) {
        v3_t r = v3_sub(right_centroid, right_pts[i]);
        v3_t l = v3_sub(left_centroid, left_pts[i]);
        
        sum_num = v3_magsq(r);
        sum_den = v3_magsq(l);
    }
    
    *scale = sqrt(sum_num / sum_den);
    
    /* Fill in the matrix M */
    for (i = 0; i < n; i++) 
    {
        v3_t r = v3_sub(right_centroid, right_pts[i]);
        v3_t l = v3_sub(left_centroid, left_pts[i]);
               
        if (weight != NULL) {
            M[0][0] += Vx(r) * Vx(l);
            M[0][1] += Vx(r) * Vy(l);
            M[1][0] += Vy(r) * Vx(l);
            M[1][1] += Vy(r) * Vy(l);
        } else {
            M[0][0] += Vx(r) * Vx(l);
            M[0][1] += Vx(r) * Vy(l);
            M[1][0] += Vy(r) * Vx(l);
            M[1][1] += Vy(r) * Vy(l);
        }
    }
    
    /* Compute MTM */
    matrix_transpose(2, 2, (double *)M, (double *)MT);
    matrix_product(2, 2, 2, 2, (double *)MT, (double *)M, (double *)MTM);
    
    /* Calculate Sinv, the inverse of the square root of MTM */
    dgeev_driver(2, (double *)MTM, (double *)evec, eval);
    
    /* MTM = eval[0] * evec[0]T * evec[0] + eval[1] * evec[1]T * evec[1] */
    /* S = sqrt(eval[0]) * evec[0]T * evec[0] + sqrt(eval[1]) * evec[1]T * evec[1] */
    sqrteval[0] = sqrt(eval[0]);
    sqrteval[1] = sqrt(eval[1]);
    
    S[0][0] = 
    (sqrteval[0]) * evec[0][0] * evec[0][0] +
    (sqrteval[1]) * evec[1][0] * evec[1][0];
    S[0][1] = 
    (sqrteval[0]) * evec[0][0] * evec[0][1] +
    (sqrteval[1]) * evec[1][0] * evec[1][1];
    S[1][0] = 
    (sqrteval[0]) * evec[0][1] * evec[0][0] +
    (sqrteval[1]) * evec[1][1] * evec[1][0];
    S[1][1] = 
    (sqrteval[0]) * evec[0][1] * evec[0][1] +
    (sqrteval[1]) * evec[1][1] * evec[1][1];
    
    Sinv[0][0] = 
    (1.0 / sqrteval[0]) * evec[0][0] * evec[0][0] +
    (1.0 / sqrteval[1]) * evec[1][0] * evec[1][0];
    Sinv[0][1] = 
    (1.0 / sqrteval[0]) * evec[0][0] * evec[0][1] +
    (1.0 / sqrteval[1]) * evec[1][0] * evec[1][1];
    Sinv[1][0] = 
    (1.0 / sqrteval[0]) * evec[0][1] * evec[0][0] +
    (1.0 / sqrteval[1]) * evec[1][1] * evec[1][0];
    Sinv[1][1] = 
    (1.0 / sqrteval[0]) * evec[0][1] * evec[0][1] +
    (1.0 / sqrteval[1]) * evec[1][1] * evec[1][1];
    
    // matrix_product(2, 2, 2, 2, (double *)S, (double *)Sinv, (double *)U);
    
    /* U = M * Sinv */
    matrix_product(2, 2, 2, 2, (double *)M, (double *)Sinv, (double *)U);
    
    /* Fill in the rotation matrix */
    R[0] = U[0][0]; R[1] = U[0][1]; R[2] = 0.0;
    R[3] = U[1][0], R[4] = U[1][1]; R[5] = 0.0;
    R[6] = 0.0;     R[7] = 0.0;     R[8] = 1.0;
    
    // memcpy(R, U, sizeof(double) * 4);
    
    /* Fill in the translation matrix */
    T[0] = T[4] = T[8] = 1.0;
    T[1] = T[3] = T[6] = T[7] = 0.0;
    T[2] = Vx(right_centroid);
    T[5] = Vy(right_centroid);
    
    Tcenter[0][0] = *scale;
    Tcenter[1][1] = *scale;
    Tcenter[0][2] = -*scale * Vx(left_centroid);
    Tcenter[1][2] = -*scale * Vy(left_centroid);
    
    matrix_product(3, 3, 3, 3, T, R, (double *)Ttmp);
    
#if 0
#if 0
    /* Do the scaling */
    Ttmp[0][0] *= *scale;
    Ttmp[0][1] *= *scale;
    Ttmp[0][2] *= *scale;
    Ttmp[1][0] *= *scale;
    Ttmp[1][1] *= *scale;
    Ttmp[1][2] *= *scale;
#else
    Tcenter[0][0] *= *scale;
    Tcenter[0][2] *= *scale;
    Tcenter[1][1] *= *scale;
    Tcenter[1][2] *= *scale;
#endif
#endif
    
    matrix_product(3, 3, 3, 3, (double *)Ttmp, (double *)Tcenter, Tout);
    
    T[2] = Vx(v3_sub(right_centroid, left_centroid));
    T[5] = Vy(v3_sub(right_centroid, left_centroid));
    
    
    /* Now compute the RMS error between the points */
    RMS_sum = 0.0;
    
    for (i = 0; i < n; i++) {
        v3_t r = v3_sub(right_centroid, right_pts[i]);
        v3_t l = v3_sub(left_centroid, left_pts[i]);
        v3_t resid;
        
        /* Rotate, scale l */
        v3_t Rl, SRl;
        
        Vx(Rl) = R[0] * Vx(l) + R[1] * Vy(l) + R[2] * Vz(l);
        Vy(Rl) = R[3] * Vx(l) + R[4] * Vy(l) + R[5] * Vz(l);
        Vz(Rl) = R[6] * Vx(l) + R[7] * Vy(l) + R[8] * Vz(l);
        
        SRl = v3_scale(*scale, Rl);
        
        resid = v3_sub(r, SRl);
        RMS_sum += v3_magsq(resid);
    }
    
    return sqrt(RMS_sum / n);
}


EpipolarGeometry::EpipolarGeometry(const std::vector<CvPoint2D32f> match_query , const std::vector<CvPoint2D32f> match_train,int num_pts, int trialFmatrix, int trialRelativepose , int focuslength, int Ransac_threshold, float threshold,int ImgWidth, int ImgHeight)
{

    //initialize all of parameters for EpipolarGeometry  
    EpipolarGeometry::num_trial_Fmatrix=trialFmatrix;
    EpipolarGeometry::num_trial_relativepose=trialRelativepose;
    EpipolarGeometry::essential= 0;
    EpipolarGeometry::Numofpts= num_pts;
    EpipolarGeometry::Ransac_threshold= Ransac_threshold;

    EpipolarGeometry::Image_Width=ImgWidth;
    EpipolarGeometry::Image_Height=ImgHeight;

   
    this->FocusLength= focuslength;
    
    r_pt = new v3_t[num_pts];
    l_pt = new v3_t[num_pts];
    
    
    for(int i=0 ; i< num_pts;i++) 
    {
        r_pt[i].p[0]=match_query[i].x;
        r_pt[i].p[1]=match_query[i].y;
        r_pt[i].p[2]=1.0;
        
        l_pt[i].p[0]=match_train[i].x;
        l_pt[i].p[1]=match_train[i].y;
        l_pt[i].p[2]=1.0;
    }
    F_matrix_threh=threshold;
}
void EpipolarGeometry::MainProcess()
{

    //FindFundamentalMatrix();
    //FindRelativePose();

}
void EpipolarGeometry::FindRelativePose(EpipolarGeometry:: Function input)
{
    //int Ransac_rounds = 200; 
    //double Ransac_threshold= 2 ;
    
    if(input == FivePoints)
    {
        Use_FivePoints= true;
    }
    else
    {
        Use_FivePoints= false;
    }    
   
    if(Use_FivePoints)
    {
        double R_out[9];
        double t_out[3];
        double camera2t[3];
        
        CenterizedFeaturePoint(lrefined_pt,rrefined_pt, num_ofrefined_pts);
        
        //double K1matrix[9];
        //double K2matrix[9];
        
        InitializeIntrinsicMatrix(K1matrix);
        InitializeIntrinsicMatrix(K2matrix); 
        
        //matrix_print(3,3,K1matrix);
        //matrix_print(3,3,K2matrix);
        
        int numinliers = compute_pose_ransac(num_ofrefined_pts, lrefined_pt, rrefined_pt, K1matrix, K2matrix, Ransac_threshold, num_trial_relativepose , R_out ,t_out);
        
        cout<<"Number of inliers " <<numinliers<<endl;
        matrix_transpose_product(3, 3, 3, 1, R_out, t_out , camera2t);
        matrix_scale(3, 1, camera2t, -1.0, t_out);
        matrix_print(3,3,R_out);
        matrix_print(3,1,t_out); 
        
        memcpy(R_relative, R_out, 9*sizeof(double));
        memcpy(t_relative, t_out, 3*sizeof(double)); 
    }
    
    if(! Use_FivePoints)
    {
        InitializeIntrinsicMatrix(K1matrix);
        InitializeIntrinsicMatrix(K2matrix); 
        
        CenterizedFeaturePoint(lrefined_pt,rrefined_pt, num_ofrefined_pts);
        v2_t* r_pts_norm = new v2_t[num_ofrefined_pts];
        v2_t* l_pts_norm = new v2_t[num_ofrefined_pts]; 
        
        double K1_inv[9];
        double K2_inv[9];
        
        matrix_invert(3, K1matrix, K1_inv);
        matrix_invert(3, K2matrix, K2_inv);
        
        bool* IndexStack= new bool [num_ofrefined_pts];
//      int NumIndex=0;
//        for(int i=0;i< num_ofrefined_pts;i++)
//        {
//            v3_t l_pt=  { Vx(lrefined_pt[i]), Vy(lrefined_pt[i]), 1.0 };
//            v3_t r_pt=  { Vx(lrefined_pt[i]), Vy(lrefined_pt[i]), 1.0 };
//            
//            double distance = fmatrix_compute_residual(Fmatrix,r_pt,l_pt);
//             if (distance<10)
//             {
//                 IndexStack[i]= true;
//                 NumIndex++;
//             }
//        }
//        
//        v2_t* r_pts_norm = new v2_t[NumIndex];
//        v2_t* l_pts_norm = new v2_t[NumIndex]; 
//        int shift =0;
//        for (int i=0;i< num_ofrefined_pts;i++)
//        {
//           // if (IndexStack[i])
//           // {
//             
//                double r[3] = { Vx(rrefined_pt[i]), Vy(rrefined_pt[i]), 1.0 };
//                double l[3] = { Vx(lrefined_pt[i]), Vy(lrefined_pt[i]), 1.0 };
//                
//                double r_norm[3], l_norm[3];
//                
//                matrix_product331(K1_inv, r, r_norm);
//                matrix_product331(K2_inv, l, l_norm);
//                
//                r_pts_norm[i] = v2_new(-r_norm[0], -r_norm[1]);
//                l_pts_norm[i] = v2_new(-l_norm[0], -l_norm[1]);
//            
//            //    shift++;
//            //}
//        }
        

        for (int i = 0; i < num_ofrefined_pts; i++) 
        {
            double r[3] = { Vx(rrefined_pt[i]), Vy(rrefined_pt[i]), 1.0 };
            double l[3] = { Vx(lrefined_pt[i]), Vy(lrefined_pt[i]), 1.0 };
            
            double r_norm[3], l_norm[3];
            
            matrix_product331(K1_inv, r, r_norm);
            matrix_product331(K2_inv, l, l_norm);
            
            r_pts_norm[i] = v2_new(-r_norm[0], -r_norm[1]);
            l_pts_norm[i] = v2_new(-l_norm[0], -l_norm[1]);
            //r_pts_norm[i] = v2_new(r_norm[0], r_norm[1]);
            //l_pts_norm[i] = v2_new(l_norm[0], l_norm[1]);
        }
        
        double tmp[9];
        double EssentialMatrix[9];
        
        matrix_transpose_product(3, 3, 3, 3, K2matrix, Fmatrix, tmp);
        matrix_product(3, 3, 3, 3, tmp, K1matrix, EssentialMatrix);

        double Rmatrix[9];
        double Tmatrix[3];
        double camera2t[3];

        find_extrinsics_essential_multipt(EssentialMatrix, num_ofrefined_pts , r_pts_norm, l_pts_norm , Rmatrix, Tmatrix);
       
        //matrix_print(3,3,Rmatrix);
       
        matrix_print(3,1,Tmatrix);
        
        matrix_transpose_product(3, 3, 3, 1, Rmatrix, Tmatrix , camera2t);
        matrix_scale(3, 1, camera2t, -1.0, Tmatrix);
        
        memcpy(R_relative, Rmatrix, 9*sizeof(double));
        memcpy(t_relative, Tmatrix, 3*sizeof(double)); 
        //cout<<""<<endl;
        delete [] r_pts_norm;
        delete [] l_pts_norm;
        delete [] IndexStack;
    
    }
}
void EpipolarGeometry::CenterizedFeaturePoint (v2_t* lrefined_pt, v2_t* rrefined_pt, int num_ofrefined_pts)
{
    for (int i=0;i<num_ofrefined_pts;i++)
    {
        v2_t p;
        v2_t q;
        
        p.p[0]= CenterX(lrefined_pt[i].p[0]); 
        p.p[1]= CenterY(lrefined_pt[i].p[1]);
        q.p[0]= CenterX(rrefined_pt[i].p[0]); 
        q.p[1]= CenterY(rrefined_pt[i].p[1]);
        
        lrefined_pt[i].p[0]=p.p[0];
        lrefined_pt[i].p[1]=p.p[1];
        rrefined_pt[i].p[0]=q.p[0];
        rrefined_pt[i].p[1]=q.p[1];
    }
    
}
void EpipolarGeometry::FindFundamentalMatrix()
{

    matched * refined_pts= new matched[1]; 
    
    //double F[9];

    F_matrix_process (this->Numofpts,  r_pt, l_pt, this-> Fmatrix , this->num_trial_Fmatrix, 10 ,  this->essential, refined_pts, 0, F_matrix_threh);
    
    
    
    num_ofrefined_pts =(int)refined_pts[0].R_pts.size();
    
    lrefined_pt= new v2_t[num_ofrefined_pts];
    rrefined_pt= new v2_t[num_ofrefined_pts];
    
    pop_backpts_WI(lrefined_pt,rrefined_pt,refined_pts,0);
    
    cout<<"Nonlinear_Fundamental_matrix"<<endl;
    matrix_print(3,3, Fmatrix);
     
    delete [] refined_pts;
}

void EpipolarGeometry::InitializeIntrinsicMatrix(double* Kmatrix)
{
    Kmatrix[0]= this-> FocusLength ,  Kmatrix[1]=0.0                , Kmatrix[2]= 0.0;
    Kmatrix[3]= 0.0                ,  Kmatrix[4]= this->FocusLength , Kmatrix[5]= 0.0;
    Kmatrix[6]= 0.0 ,                 Kmatrix[7]= 0.0               , Kmatrix[8]= 1.0;

}

void EpipolarGeometry:: FindApicalAngle (float MaxAngle)
{
     
    ApicalAngle =  Apical_Angle(lrefined_pt, rrefined_pt, R_relative , t_relative , K1matrix, num_ofrefined_pts); 
     
    if(ApicalAngle>= MaxAngle)
        this->skipFrame=0;
    else 
        this->skipFrame=1;

}

EpipolarGeometry::~EpipolarGeometry()
{
    
    delete []  r_pt;
    delete []  l_pt;
    delete []  lrefined_pt;
    delete []  rrefined_pt;

}

void EpipolarGeometry::InitializeFirstPmatrix()
{
    
    R1matrix[0] = 1.0;  R1matrix[1] = 0.0;  R1matrix[2] = 0.0;
    R1matrix[3] = 0.0;  R1matrix[4] = 1.0;  R1matrix[5] = 0.0;
    R1matrix[6] = 0.0;  R1matrix[7] = 0.0;  R1matrix[8] = 1.0; 

    t1matrix[0] = 0.0;  t1matrix[1] = 0.0;  t1matrix[2] = 0.0;
} 

void EpipolarGeometry::TwoviewTriangulation(vector<v2_t> & left_pts,vector<v2_t> & right_pts, vector<v3_t> & V3Dpts)
{
    TwoviewTria=1;
    int size_= num_ofrefined_pts;
    v3_t m_3Dpts [size_];
    
    double error_tr=0.0;
      
      for (int i=0; i< size_ ;i++)
         {
             bool in_front = true;
             double angle = 0.0;
             v3_t temp; 
             v2_t p;
             v2_t q;

             
             p.p[0] = lrefined_pt[i].p[0];
             p.p[1] = lrefined_pt[i].p[1];
             q.p[0] = rrefined_pt[i].p[0];
             q.p[1] = rrefined_pt[i].p[1]; 
             
             temp = Triangulate(p, q, R1matrix , t1matrix ,R_relative, t_relative, error_tr, in_front, angle ,true,K1matrix,K2matrix);  
            
             m_3Dpts[i]= temp;
             //printf("%0.4f %0.4f %0.4f\n", temp.p[0], temp.p[1],temp.p[2]);
          }

    PointRefinement(m_3Dpts, left_pts , right_pts, V3Dpts);
     
}


void EpipolarGeometry::TwoviewTriangulation_1(vector<v2_t> & left_pts,vector<v2_t> & right_pts, vector<v3_t> & V3Dpts)
{
    TwoviewTria=1;
    int size_= num_ofrefined_pts;
    v3_t m_3Dpts [size_];
    
    double error_tr=0.0;
    
    for (int i=0; i< size_ ;i++)
    {
        bool in_front = true;
        double angle = 0.0;
        v3_t temp; 
        v2_t p;
        v2_t q;
        p.p[0] = lrefined_pt[i].p[0];
        p.p[1] = lrefined_pt[i].p[1];
        q.p[0] = rrefined_pt[i].p[0];
        q.p[1] = rrefined_pt[i].p[1]; 
        
        temp = Triangulate(p, q, R1matrix , t1matrix ,R_relative, t_relative, error_tr, in_front, angle ,true,K1matrix,K2matrix);
        
        m_3Dpts[i]= temp;
        V3Dpts.push_back(m_3Dpts[i]);
        printf("%0.4f %0.4f %0.4f\n", temp.p[0], temp.p[1],temp.p[2]);
    }
    
    PointRefinement(m_3Dpts, left_pts , right_pts, V3Dpts);
    
}


void EpipolarGeometry::PointRefinement(v3_t* m_3Dpts,vector<v2_t> & left_pts,vector<v2_t> & right_pts, vector<v3_t> & V3Dpts)
{
    
    bool* tempvector = new bool [num_ofrefined_pts];
    
    for (int i=0;i< num_ofrefined_pts;i++)
           tempvector[i]= false;

    /// check Cheirality
    //    for (int i=0;i<num_ofrefined_pts;i++)
    //    {
    //       if(CheckCheirality(m_3Dpts[i]))
    //       { 
    //           tempvector[i]= true;
    //       }
    //    }
    
    _3DdepthRefine ( m_3Dpts, tempvector, num_ofrefined_pts);
    
    int Numindex=0;
    
    for (int i=0;i<num_ofrefined_pts;i++)
    {
        if(! tempvector[i])
            Numindex++;
     }
   
    //v3_t* new3Dpts    = new v3_t [Numindex];
    //v2_t* newLeftptas = new v2_t [Numindex];
    //v2_t* newRightpts = new v2_t [Numindex];
    
    int index=0;
    
    for (int i=0;i<num_ofrefined_pts;i++)
    {
        if(! tempvector[i])
        {
            V3Dpts.push_back(m_3Dpts[i]);
            left_pts.push_back(lrefined_pt[i]);
            right_pts.push_back(rrefined_pt[i]);
            index++;
        }
    }
    
    num_ofrefined_pts= index;

    //for (int i=0; i< num_ofrefined_pts ;i++)
    //    printf("%0.4f %0.4f %0.4f\n", V3Dpts[i].p[0],V3Dpts[i].p[1],V3Dpts[i].p[2]);
   
    delete  []  tempvector;
    //delete  []  new3Dpts;
    //delete  []  newLeftptas;
    //delete  []  newRightpts;

}
 void EpipolarGeometry::_3DdepthRefine (v3_t* m_3Dpts, bool* tempvector, int num_ofrefined_pts)
{
    
    int size_= num_ofrefined_pts;
    double max_number =  999;    // remove outliers from candidated points 
    double min_number = -999;
    double range;
    int i;
    int Nbins = 101;

    int Bin[101]={};
  
    int mx_index= 0;
    double  Range_low;
    double  Range_upper;
    
    double maxDepth = 0.0;
    double minDepth =0.0;
    
    // Remove some incredible depth first //
    
    for (int i=0;i< size_;i++)
    {
             if ( m_3Dpts[i].p[2]< min_number)
                   tempvector[i]= true;
             if ( m_3Dpts[i].p[2]> max_number)
                   tempvector[i]= true;
    }
    
    // find max and min for voting 
    for (int i=0;i< size_;i++)
    {
         if( !tempvector[i] ) 
         {
           if( m_3Dpts[i].p[2]> min_number)
           {  
               maxDepth = m_3Dpts[i].p[2];
               min_number= maxDepth;
           }
            if( m_3Dpts[i].p[2]< max_number)
             {
                 minDepth= m_3Dpts[i].p[2] ;
                 max_number= minDepth;
             }
         }
        
    }
    if (maxDepth >(abs(minDepth)))
    {
        maxDepth=(abs(minDepth));
    }
    range = fabs((maxDepth - minDepth) / Nbins);
  for(i=0; i<size_;i++)
    {
       if (tempvector[i] == false)
       {

            float x = (float) m_3Dpts[i].p[2];
            int idx = round(((maxDepth-x)/(maxDepth-minDepth))*Nbins);
            Bin[idx] += 1;
          
                

       }
    }


    int mx_bin  = 0;
    for (int i=0;i<Nbins;i++)
    {
     if(Bin[i]> mx_bin)
       {
        mx_bin = Bin[i];
        mx_index = i;
        
       }
    }

    float depth = -(((mx_index)*range)-maxDepth);
    
    float varince= Variance (m_3Dpts, depth, size_);

    float densitytemp;
    cout<< depth <<"variance "<<varince <<endl;
    for (int i=0;i< size_;i++)
    {
         float x = (float) m_3Dpts[i].p[2];
         float a=-fabs(x-depth)*(1./(1.06*(sqrt(varince))*2.1));
          float density = exp(a);
         densitytemp =exp(a);
        if (densitytemp <0.3)
            tempvector[i] = true;
    }

    //delete [] densitytemp;
}


float  EpipolarGeometry:: Variance (v3_t* _3Dpts, const float depth , const int size_)
{   
    float* tempz= new float [size_];
    float sum=0;
    int num=0;
    for (int i=0;i<size_;i++)
    { 
        if(_3Dpts[i].p[2]< 0 && _3Dpts[i].p[2]> 2.5 *depth)
        {
          tempz[i]= (float)(_3Dpts[i].p[2]-depth)*(_3Dpts[i].p[2]-depth);
          sum=sum+tempz[i];
          num++;
        }
    }
    
    return( sum *= 1. / num );

}
IplImage* EpipolarGeometry:: plot_two_imagesf(IplImage *IGray, IplImage *IGray1)
{
    CvPoint newmatched;       
    CvPoint matched;
    int  ttw  =  IGray->width+ IGray->width;
    int  ttl  =  IGray->height;
    IplImage* Imagedisplay  = cvCreateImage(cvSize(2*IGray->width,IGray->height), IGray->depth, IGray->nChannels );
    for (int i=0; i<ttl;i++)
    {
        for (int j=0; j<ttw; j++)
        {
            if (j< IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray,i,j));
            }
            if (j>IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray1,i,j-(IGray1->width)));
            }
        }
    }
    
    CvPoint pt1,pt2;
    pt1.x=0;
    pt1.y=0;
    pt2.x= (IGray->width)*2;
    pt2.y= (IGray->height);
    
    if(skipFrame)
        cvRectangle(Imagedisplay,pt1 , pt2, CV_RGB(256,0,0), 4, 8, 0 );
    else 
        cvRectangle(Imagedisplay,pt1 , pt2, CV_RGB(0,256,0), 4, 8, 0 );
        for (int i=0;i<num_ofrefined_pts; i++)
    {           
        newmatched.x= (int)(lrefined_pt[i].p[0])+(IGray->width)+(0.5*(IGray->width));
        newmatched.y= (int)(lrefined_pt[i].p[1])+(0.5*(IGray->height));
        matched.x = (int) rrefined_pt[i].p[0]+(0.5*(IGray->width));
        matched.y = (int) rrefined_pt[i].p[1]+(0.5*(IGray->height));
        
        cvLine(Imagedisplay, 
        cvPoint( matched.x, matched.y ), 
        cvPoint( newmatched.x, newmatched.y ), 
        CV_RGB(256,256,256)
        );
    

        cvCircle(Imagedisplay, newmatched, 3, CV_RGB(0,255,0),2,6,0);
        cvCircle(Imagedisplay, matched, 3, CV_RGB(0,255,0), 2,6,0);
        
    }  
    return(Imagedisplay);
}

//void EpipolarGeometry::ReprojionErrorRefin (v3_t* m_3Dpts, bool* tempvector, int num_ofrefined_pts)
//{
//    
//    int size_= num_ofrefined_pts;
//    double max_number = -1.0;    // remove outliers from candidated points 
//    double min_number = -50.0;
//    double range;
//    int i;
//    int Nbins = 50;
//    
//    int Bin[50]={};
//    
//    int mx_index= 0;
//    double  Range_low;
//    double  Range_upper;
//    
//    for (int i=0;i< size_;i++)
//    {
//        if ( m_3Dpts[i].p[2]< min_number)
//            tempvector[i]= true;                 
//    }
//    
//    range = (max_number - min_number) / Nbins;
//    for(i=0; i<size_;i++)
//    {
//        if (tempvector[i] == false)
//        {
//            for(int index=0; index< Nbins; index++)
//            {
//                
//                Range_low = min_number   +   (index)*range;
//                Range_upper = min_number +   (index+1)*range;
//                float x = (float) m_3Dpts[i].p[2];
//                
//                if ( Range_low < x && x <= Range_upper)              
//                {
//                    Bin[index] += 1; 
//                    
//                }
//            }
//        }
//    }
//    int mx_bin  = 0;
//    for (int i=0;i<Nbins;i++)
//    {
//        if(Bin[i]> mx_bin)
//        {
//            mx_bin = Bin[i];
//            mx_index = i;
//            
//        }
//    }
//    
//    float depth = (min_number+(mx_index)*range);
//    float varince= Variance (m_3Dpts, depth, size_);
//    float *densitytemp  = new float [size_]; 
//    cout<< depth <<"variance "<<varince <<endl;
//    for (int i=0;i< size_;i++)
//    {
//        float x = (float) m_3Dpts[i].p[2];
//        float a=-fabs(x-depth)*(1./(1.06*(sqrt(varince))*2.1));
//        //float density = exp(a);
//        densitytemp[i]=exp(a);
//        if (densitytemp[i]<0.3)
//            tempvector[i] = true;
//    }
//    delete [] densitytemp;
//}

