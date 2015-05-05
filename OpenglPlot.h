//
//  OpenglPlot.h
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/24/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//
#include "glfw3.h"
#include <vector.h>
#include "vector.h"
#include "OpenGLMatrices.h"
#include <math.h>



//float near=5;
//float far=700;


#define M_pi 3.1416


//float Mouse_r_theta=0;
//float Mouse_y_theta=0;
//float Mouse_p_theta=0;
//



Matrix4X4 setFrustum(float l, float r, float b, float t, float n, float f);
Matrix4X4 setFrustum(float fovY, float aspectRatio, float front, float back);


class OpenGLPlot 
{
    
public:
    int width;
    int height;
  
    float Dnear;
    float Dfar;
    double theta;
    double f_Distance;
    
    float center_x;
    float center_y;
    float center_z;
    
    float Mouse_startx;
    float Mouse_starty;

    float Mouse_r_theta;
    float Mouse_y_theta;
    float Mouse_p_theta;
    
    GLFWwindow* window;
    GLFWwindow* window2;
    OpenGLPlot (int width , int height);
    ~ OpenGLPlot ();
    
    void error_callback(int error, const char* description);
    //void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    void Initialized();
    void Setview(const vector<v3_t> V3Dpts);
    void PlotVertex(int size_, const vector<v3_t> V3Dpts);
    void PlotCamera(double* t_relative);
    void SetupFrustrum();
    void Reshape(int height, int width );
    void PlotVertex2(int size_, const vector<v3_t> V3Dpts);

    void ReadObject(const vector<v3_t> V3Dpts);
    void  Setrot();
    void SetCamera();
private:
    
};

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
