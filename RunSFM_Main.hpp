#ifndef __RunSFM_Main_h__
#define __RunSFM_Main_h__

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "vector.h"
#include "sfm.h"
#include <vector>
# include "Relative_Pose.h"
using namespace std;

#define NUM_CAMERA_PARAMS 9
#define POLY_INVERSE_DEGREE 6
double ReprojectError( double *R, double* Tc, v3_t Pts, v2_t Projpts, double * Kmatrix);


void InitializeCameraParams(camera_params_t &camera);


double RunSFM_Nviews_Main(int num_pts /*number of 3D pts */, 
                          int num_cameras, 
                          int start_camera,                   
                          vector<RotMat>&  mtriRotmatrix,     /*camera rotation matrix*/
                          vector<TMat>&    mtriTcmatrix,      /*camera translation matrix*/
                          vector<Kmat>&    mtriKmatrix,       /*camera instrinstic matrix*/ 
                          vector<vector<v2_t> >& mv2_location /*2D points location*/ , 
                          vector<vector<int> >&  mv2_frame    /*frame number*/, 
                          vector<v3_t>& v3Pts                /*triangulation output*/);

double RunSFM_Nviews(int num_pts, int num_cameras, int start_camera, int Numofframe, 
                     camera_params_t *init_camera_params, v3_t* sfm3Dpts ,
                     char* vmask, double* projections,  
                     bool EstimateFocal, bool UseFocalconstraints, 
                     bool fix_points, bool UsePointConstraint,double eps2, 
                     double *S, double *U, double *V, double *W,
                     int NumIteration,bool remove_outliers);


void SetCameraConstraints(camera_params_t params, bool estimate_distoration); 
void SetFocalConstraint( camera_params_t params);
void InitializedCameraParameters ( 
                             int i,         
                             vector<RotMat>  mtriRotmatrix,     /*camera rotation matrix*/
                             vector<TMat>    mtriTcmatrix,      /*camera translation matrix*/
                             vector<Kmat>    mtriKmatrix,
                             camera_params_t* CameraPara
                                  );

class CameraInfo {
    
public:
    bool m_constrained[7];
    double m_constraints[7];
    double m_constraint_weights[7];
    CameraInfo() 
    {
        m_constrained[0] = m_constrained[1] = m_constrained[2] = false; 
        m_constrained[3] = m_constrained[4] = m_constrained[5] = false; 
        m_constrained[6] = false;
        m_constraints[0] = m_constraints[1] = m_constraints[2] = 0.0;
        m_constraints[3] = m_constraints[4] = m_constraints[5] = 0.0;
        m_constraints[6] = 0.0;
        m_constraint_weights[0] = 
        m_constraint_weights[1] = 
        m_constraint_weights[2] = 0.0;
        m_constraint_weights[3] = 
        m_constraint_weights[4] = 
        m_constraint_weights[5] = 0.0;
        m_constraint_weights[6] = 0.0;      
        
        //       m_k[0] = m_k[1] = 0.0;
        //#ifndef __BUNDLER__
        //        for (int i = 0; i < NUM_LINK_DIRECTIONS; i++) 
        //            m_links[i] = -1;
        //#endif
    }
};


#endif 