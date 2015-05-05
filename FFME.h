//-------------------------------------------------------------------------
// FFME.h  
//-------------------------------------------------------------------------
// Description: class for image motion estimation based on features. This 
// software is based on the paper: "Motion estimation through efficient 
// matching of a reduced number of reliable singular points". 
//-------------------------------------------------------------------------
// Author: Carlos Roberto del Blanco Ad�n, 
//         Grupo de Tratamiento de Im�genes, GTI SSR, Madrid
//                 cda@gti.ssr.upm.es
// Date: 2008
// Version 1.0
//-------------------------------------------------------------------------

//OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

//Others
//#include "miscellaneous.h"


//********************************************Parameters******************************************
//Feature selection parameters.
#define THRESH_GRAD_MAG 100 //Threshold for the gradient magnitude image.
#define THRESH_HARRIS 10 //Cornerness threshold based on Harris condition (ratio of eigenvalues).
#define WIDTH_WIN_HARRIS 7 //Size of the window used to calculate the cornerness.
#define WIDTH_WIN_NONMAXSUP 7 //Size of the window used to calculate the non-maximal supression. 


//Descriptor parameters.
#define WIDTH_ARRAY_HIST 4 //Width of the square array of orientations histograms.
#define WIDTH_SUBWIN_HIST 4 //Width in pixels of the sub-windows where a orientation histogram is computed.
#define NO_BINS_ORI_HIST 8 //Number of bins of each orientation histogram.
#define MAX_RESP_COMP_DESC 0.2 //Maximum value allowed for each component of the vector descriptor. It provides robustness to non-affine ilumination changes.

//Matching parameters.
#define THRESH_RATIO_SECOND_BEST_CORR 0.49 //Second nearest neighbor restriction. This restriction removes correspondences
                                                                                   //if the ratio of descriptor distances between the first and the second best correspondence 
                                                                                   //of a feature point is higher than a threshold. Determine the reliability of the correspondence.
#define RAD_MAX_SEARCH 16 //Maximum radius of search in the correspondence process.
//************************************************************************************************

//***********************************Macros***********************************************
// It is recommended to put into brackets all the arguments to avoid problems in the case that
// one argument will be a mathematical expresion. 

// Access to the image pixels. (i, j) = (row, col).
#define pixelImgU81C_M(img_U81C, i, j)  (((unsigned char*)((img_U81C)->imageData+(i)*(img_U81C)->widthStep))[(j)])
#define pixelImgS161C_M(img_S161C, i, j)  (((short*)((img_S161C)->imageData+(i)*(img_S161C)->widthStep))[(j)])
#define pixelImg32F1C_M(img_32F1C, i, j)  (((float*)((img_32F1C)->imageData+(i)*(img_32F1C)->widthStep))[(j)])

// Access to the matrix elements. (i, j) = (row, col).
#define elemMat32F1C_M(mat_32F1C, i, j)  (((float*)((mat_32F1C)->data.ptr+(i)*(mat_32F1C)->step))[(j)])

// Access to the tridimensional matrix elements. (i, j) = (row, col).
#define elemMat3D32F_M(mat_3D32F, i, j, l)  (((float*)((mat_3D32F)->data.ptr+(i)*((mat_3D32F)->dim[0].size*4)+(j)*((mat_3D32F)->dim[1].size*4))[(l)])

// Check that the coordinates (row,col) belong to the image.
#define checkMargins(image, row, col) (row) >= 0 && (row) < (image)->height && (col) >= 0 && (col) < (image)->width 

// Check that the squared region defined by its center (row,col) and its half width (hwidth) belongs to the image.
#define checkSquareReg(image, row, col, hwidth) ((row)-hwidth) >= 0 && ((row)+hwidth) < (image)->height && ((col)-hwidth) >= 0 && ((col)+hwidth) < (image)->width
//****************************************************************************************

// Converts an image of type XX1C to U81C and scales the intensity values to show the maximum dynamic range of the image. 
void scaleXX1CtoU81C(IplImage* imgXX1C, IplImage* imgU81C);

// Draws an array of points as white points in a black and white image.
void drawPtosBW(IplImage* imgU81C, CvPoint2D32f* ptos, int noPtos);


// Print the values of a float vector.
void printVectorFloat(float* vector, int length, int newline);

// Draw singular points on an image.
void drawPtosIm(IplImage* imgU83C, CvPoint2D32f* singPtos, int noSingPtos);

// Draw correspondences as a sparse motion vector field.
void drawCorr(IplImage* imgU83C, CvPoint2D32f** correspondences, int noCorr);

//Print correspondence vector values.
void printCorr(CvPoint2D32f** correspondences, int noCorr);    

class FFME
{
//Methods:
public:
        FFME(void);
        ~FFME(void);

	// OpenCV 2.x C++ API wrapper 
	void init(cv::Size size, int maxNoKeyPoints);
	void detect(cv::Mat &img, std::vector<cv::KeyPoint> &query_kpts);
	void describe(cv::Mat &img, std::vector<cv::KeyPoint> &query_kpts, cv::Mat &query_desc);
	void match(std::vector<cv::KeyPoint> &query_kpts, std::vector<cv::KeyPoint> &train_kpts, std::vector<cv::DMatch> &matches);
	void updateBuffers();
	bool started() { return m_started; }

        //Initialization of memory and look-up tables.
        void iniFFME(int width, int height, int origin, int maxNoKeyPoints);
        //Singular point detection based on functions.
        void singPtoDetFunc(IplImage* img_U81C, CvPoint2D32f* singPtos, int* noSingPtos);
        //Singular point detection based on LUT.
        void singPtoDetLut(IplImage* img_U81C, CvPoint2D32f* singPtos, int* noSingPtos);
        //Singular point description based on functions.
        void singPtoDescFunc(CvPoint2D32f* singPtos, int noSingPtos, float** descriptors, bool normDesc = true);
        //Singular point description based on LUT.
        void singPtoDescLut(CvPoint2D32f* singPtos, int noSingPtos, float** descriptors, bool normDesc = true);
        //Matching of singular points.
        void matchSingPtos(CvPoint2D32f* singPtos1, int noSingPtos1, float** descriptors1, 
                               CvPoint2D32f* singPtos2, int noSingPtos2, float** descriptors2, 
                                           CvPoint2D32f** correspondences, int* noCorr);

        //--get and set functions--//
        //Set feature selection parameters.
        void setFeatParam(float threshGradMag, float threshHarris, int widthWinHarris, int widthWinNonMaxSup)
        {
                m_threshGradMag = threshGradMag;
                m_threshHarris = threshHarris;
                m_widthWinHarris = widthWinHarris;
                m_widthWinNonMaxSup = widthWinNonMaxSup;
        }

        //Set descriptor parameters.
        void setDescParam(int widthArrayHist, int widthSubWinHist, int noBinsOriHist, float maxRespCompDesc)
        {
                m_widthArrayHist = widthArrayHist;
                m_widthSubWinHist = widthSubWinHist;
                m_noBinsOriHist = noBinsOriHist;
                m_maxRespCompDesc = maxRespCompDesc;
        }

        //Set matching parameters.
        void setMatchParam(float threshRatSecBest, float radMaxSearch)
        {
                m_threshRatSecBest = threshRatSecBest;
                m_radMaxSearch = radMaxSearch;
        }

        //Get horizontal gradient.
        void getHorGradient(IplImage** img_S161C)
        {
                *img_S161C = m_horGradient_S161C;
        }

        //Get vertical gradient.
        void getVerGradient(IplImage** img_S161C)
        {
                *img_S161C = m_verGradient_S161C;
        }

        //Get gradient magnitud.
        void getGradMag(IplImage** img_32F1C)
        {
                *img_32F1C = m_magGradient_32F1C;
        }

        //Get the number of points that have fullfiled the gradient magnitud restriction.
        void getNoGradPtos(int* noPtosGrad)
        {
                *noPtosGrad = m_noPtosGrad;
        }

        //Get the points that have fullfiled the gradient magnitud restriction.
        void getGradPtos(CvPoint2D32f** ptosGrad)
        {
                *ptosGrad = m_ptosGrad;
        }

        //Get the number of points that have fullfiled the cornerness restriction.
        void getNoCornerPtos(int* noPtosCorner)
        {
                *noPtosCorner = m_noPtosCornerness;
        }

        //Get the points that have fullfiled the cornerness restriction.
        void getCornerPtos(CvPoint2D32f** ptosCorner)
        {
                *ptosCorner = m_ptosCornerness;
        }

private:
        /*--Funtions related to singular point detection--*/
        //Compute the horizontal and vertical image gradient based on Sobel.
        void gradientSobel(IplImage* img_U81C);
        //Compute the gradient magnitude based on functions.
        void gradMagFunc();
        //Compute the gradient magnitude based on LUT.
        void gradMagLut();
        //Selection of points by gradient magnitude thresholding. The 'noPix' nearer from image borders are discarded.
        void gradMagThresh(float thresh, int noPix);
        //Selection of points of high cornerness.
        void cornerThresh(float thresh, int sizeWin);
        //Non minimal supression in the cornerness space.
        void nonMinSupCorner(int sizeWin, CvPoint2D32f* ptos, int* noPtos);

        /*--Funtions related to singular point description--*/
        //Compute the gradient phase based on functions. Range = (0,2pi).
        void gradPhaseFunc();
        //Compute the gradient phase based on LUT. Range = (0,2pi).
        void gradPhaseLut();
        //Compute the orientations histograms based on funtions.
        void orientHist(CvPoint2D32f* singPto, float* descriptor);
        //Orientation histogram contributions by trilinear interpolation.
        void trilinearInterp(float rbin, float cbin, float obin, float gradVal, float* descriptor);
        //Normalization of descriptor vector (to be invariant to illumnination changes).
        void normDescrip(float* descriptor, int length, float maxRespComp);     
        //Vector normalization for float type.
        void normVector(float* vector, int length);

        /*--Funtions related to singular point correspondence--*/
        //Euclidean distance of two vectors of type float.
   void euclDist(float* vector1, float* vector2, int length, float* dist);

//Members:
public:
        //--Feature selection parameters--//
        //Maximum number of keypoints that can be detected.
        int m_maxNoKeyPoints;
        //Threshold for the gradient magnitude image.
        float m_threshGradMag; 
        //Cornerness threshold based on Harris condition (ratio of eigenvalues).
        float m_threshHarris; 
        //Size of the window used to calculate the cornerness. 
        int m_widthWinHarris; 
        //Size of the window used to calculate the non-maximal supression.
        int m_widthWinNonMaxSup;

        //--Descriptor parameters--//
        int m_widthArrayHist; //Width of the square array of orientations histograms.
        //Width in pixels of the sub-windows where a orientation histogram is computed.
        int m_widthSubWinHist;
        //Number of bins of each orientation histogram.
        int m_noBinsOriHist;
        //Maximum value allowed for each component of the vector descriptor.
        float m_maxRespCompDesc;

        //--Matching parameters--//
        //Second nearest neighbor restriction. This restriction removes correspondences if the ratio of descriptor distances between the first
        //and the second best correspondence of a feature point is higher than a threshold. Determine the reliability of the correspondence.
        float m_threshRatSecBest;
        //Maximum radius of search in the correspondence process.
        float m_radMaxSearch; 
private:
        //Horizontal gradient image.
        IplImage* m_horGradient_S161C;
        //Vertical gradient image.
        IplImage* m_verGradient_S161C;
        //Magnitude gradient image.
        IplImage* m_magGradient_32F1C;
        //Phase gradient image.
        IplImage* m_phaseGradient_32F1C;
        //Sparse image cornerness.
        IplImage* m_cornerness_32F1C;
        
        /*Look-up tables*/
        float** m_LutMagGradient;
        float** m_LutPhaseGradient;
        
        //Array of points that fullfill the gradient magnitude restriction.
        CvPoint2D32f* m_ptosGrad;
        //Number of points that fullfill the gradient magnitude restriction.
        int m_noPtosGrad;
        //Array of points that fullfill the cornerness restriction.
        CvPoint2D32f* m_ptosCornerness;
        //Number of points that fullfill the cornerness restriction.
        int m_noPtosCornerness;

	// C++ OpenCV 2.x API Wrapper variables
	CvPoint2D32f* singPoints1;
        CvPoint2D32f* singPoints2;
        CvPoint2D32f* tmpPoints;
	int noSingPoints1;
        int noSingPoints2;
        float** descriptors1;
        float** descriptors2;
        float** tmpDesc;
        int lengthDesc;
        int noCorr;
        CvPoint2D32f** correspondences;

	bool m_started;
};


