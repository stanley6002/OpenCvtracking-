//
//  Relative_Pose.h
//  Optical_flow 
//
//  Created by chih-hsiang chang on 4/1/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

# include <vector>
# include <iostream>
# include "matrix.h"
# include "vector.h"
# include "matrix.h"
# include "F_matrix.h"
# include "FeaturePoint.h"
# include  "CameraPoseRefinement.h"

//#include "FeaturePoint.h"

double ReprojectError( double *R, double* Tc, v3_t Pts, v2_t Projpts, double * Kmatrix);

void DumpPointsToPly(char *output_directory, vector<v3_t> points
                     ,int num_points) ;

bool CheckCheirality(v3_t _3DPts);

float Variance (vector<v3_t> m_3Dpts, const  float depth , const int size_);

void  _3DdepthRefine (vector<v3_t> m_3Dpts, vector<bool>& tempvector, int num_ofrefined_pts);

void RefineN_FramePoints(vector<v3_t> _3DPts, int NumPts, vector<bool>& tempvector);

float  Variance (vector<v3_t> _3Dpts, const float depth , const int size_);

bool CheckCheirality(v3_t pt);



using namespace std;
typedef struct 
{
    double n[9];
} Kmat;
typedef struct 
{
    double n[9];
    
}  RotMat;

typedef  struct 
{
    
    double n[3];
    
} TMat;

double ReprojectionError();

class CameraPose
{
    //friend class EpipolarGeometry;
    
    public:
    vector <RotMat> mRcMatrix;
    vector <TMat>   mTcMatrix;
    vector <Kmat>   KMatrix;
    
    vector<RotMat>  mtriRotmatrix;
    vector<TMat>    mtriTcmatrix;
    vector<Kmat>    mtriKmatrix;

       
    inline void LoadTriRotmatrix(double* R )
    {
        RotMat tempR;
        memcpy( tempR.n , R, 9* sizeof(double));
        mtriRotmatrix.push_back(tempR);
    }
    
    inline void LoadTriTcmatrix(double* T)
    {
        TMat tempT;
        memcpy( tempT.n , T, 3* sizeof(double));
        mtriTcmatrix.push_back(tempT);
    }
    
    inline void LoadTriKmatrix(double* K, int i)
    {
        mtriKmatrix[i].n[0]= K[0];  mtriKmatrix[i].n[1]= K[1];  mtriKmatrix[i].n[2]= K[2];
        mtriKmatrix[i].n[3]= K[3];  mtriKmatrix[i].n[4]= K[4];  mtriKmatrix[i].n[5]= K[5];
        mtriKmatrix[i].n[6]= K[6];  mtriKmatrix[i].n[7]= K[7];  mtriKmatrix[i].n[8]= K[8];  
    }

    inline void PopTriKMattix(int i, double*K)
    {
        //cout<<i<<endl;
        memcpy(K,mtriKmatrix[i].n, 9*sizeof(double));
    }
    
    inline void PopTriRotcMatrix(int i, double* R)
    {
        memcpy(R,mtriRotmatrix[i].n, 9*sizeof(double));
    }
    
    inline void PopTriTcMatrix(int i, double* T)
    {
        memcpy(T,mtriTcmatrix[i].n,3*sizeof(double));
    }
    
    inline void InitializeFirstTwoKMatrix(double* K1Matrix, double*K2Matrix)
    {
        Kmat temp1;
        Kmat temp2;
        
        memcpy( temp1.n , K1Matrix, 9* sizeof(double));
        KMatrix.push_back(temp1);
        mtriKmatrix.push_back(temp1);
        
        memcpy( temp2.n , K2Matrix, 9* sizeof(double));
        KMatrix.push_back(temp2);
        mtriKmatrix.push_back(temp2);
    }
    
    inline void InitializeKMatrix (int Foucslength)
    {
        Kmat temp ;
        
        temp.n[0]= Foucslength,          temp.n[1]=0.0                , temp.n[2]= 0.0;
        temp.n[3]= 0.0                ,  temp.n[4]= Foucslength ,       temp.n[5]= 0.0;
        temp.n[6]= 0.0 ,                 temp.n[7]= 0.0               , temp.n[8]= 1.0;
    
        KMatrix.push_back(temp);
        mtriKmatrix.push_back(temp);
    }
    
    inline void LoadRotcMatrix(double* R )
    {
        RotMat tempR;
        memcpy( tempR.n , R, 9* sizeof(double));
        mRcMatrix.push_back(tempR);
    }
    
    inline void LoadTcMatrix(double* T)
    {
        TMat tempT;
        memcpy( tempT.n , T, 3* sizeof(double));
        mTcMatrix.push_back(tempT);
    }
    inline void LoadKMatrix(double* K, int i)
    {
          KMatrix[i].n[0]= K[0];  KMatrix[i].n[1]= K[1];  KMatrix[i].n[2]= K[2];
          KMatrix[i].n[3]= K[3];  KMatrix[i].n[4]= K[4];  KMatrix[i].n[5]= K[5];
          KMatrix[i].n[6]= K[6];  KMatrix[i].n[7]= K[7];  KMatrix[i].n[8]= K[8];  
    }
    inline void First2viewInitialization(double* R1, double* R_relative, double* T1, double* T_relative)
    {
        LoadRotcMatrix(R1);
        LoadRotcMatrix(R_relative);
        
        LoadTriRotmatrix(R1);
        LoadTriRotmatrix(R_relative);
        
        LoadTcMatrix(T1);
        LoadTcMatrix(T_relative);
        
        LoadTriTcmatrix(T1);
        LoadTriTcmatrix(T_relative);
    }
    
    inline void RemoveTri()
    {
    
        mtriRotmatrix.erase(mtriRotmatrix.begin(), mtriRotmatrix.begin()+1);
        mtriTcmatrix.erase(mtriTcmatrix.begin(), mtriTcmatrix.begin()+1);
        mtriKmatrix.erase(mtriKmatrix.begin(), mtriKmatrix.begin()+1);
    }
    
    inline void PopKMattix(int i, double*K)
    {
         memcpy(K,KMatrix[i].n, 9*sizeof(double));
    }
    
    inline void PopRotcMatrix(int i, double* R)
    {
        memcpy(R,mRcMatrix[i].n, 9*sizeof(double));
    }
    
    inline void PopTcMatrix(int i, double* T)
    {
        memcpy(T,mTcMatrix[i].n,3*sizeof(double));
    }
    
    // inline void LoadRelativePose(double* R_relative, double* T_relative)
    //  {
    //    LoadTcMatrix(T_relative);
    //    LoadRotcMatrix(R_relative);
    // }
   
    inline int SizeofPose()
    {   
        if (mRcMatrix.size() != mTcMatrix.size())
            
        {  
             cout<<"size inconsistent "<<endl;
             }
        
        return((int) mRcMatrix.size());
    }

    void Egomotion(double *R_relative, double* T_relative , vector<v3_t> v3ProjectPts, vector<v2_t> v2ReprojectPts );
    
    void PrintRotmatrix(int i);
    
    void PrintTmatrix(int i);
    
    void PrintKmatrix(int i);
    
    void TwoDalighment( int NumofProject, double*Rot, double*trans, v3_t* P__3DSolvedforparameters, 
                       v2_t* P__2DSolvedforparameters);
  
    
    double CameraReprojectError(int NumPts, double *R, double* Tc, v3_t* Pts, v2_t* Projpts, double * Kmatrix);
    
    //double TriangulationN_Frames(FeaturePts* Pts);
   
    void TriangulationN_Frames(vector<vector<v2_t> > mv2_location /*2D points location*/ ,  vector<vector<int> >  mv2_frame /*frame number*/, 
               vector<v3_t>& v3Pts/*triangulation output*/ , 
               vector<bool>&  tempvector /*save array for refinement*/); 
    
    double CameraReprojctionErrorRefinement(vector<vector<v2_t> > mv2_location /*2D points location*/ ,  vector<vector<int> >  mv2_frame /*frame number*/, int Numpts, vector<v3_t> v3Pts,  vector<bool>&  tempvector );
    
    double ReprojectionError(double *R, double* Tc, v3_t Pts, v2_t Projpts, double * Kmatrix);
    //inline void PopRotmatirx(double* R);
    //inline void PopTmatrix(double* T);
    
    CameraPose ();
    ~CameraPose ();
    
    private:
    
      void   deltavector(v2_t* _2Dpt,  v3_t* _3Dpt, double* Parametrvec_);
      double Euclidence_3D (v2_t _2Dpt, v3_t _3Dpt, double* Parameter_vec);
      void   DeltaVector_Ransac(v2_t* _2Dpt, v3_t* _3Dpt, int size_ , double *       Parameter_vec, int Ransac_runs, double error);

    
};
