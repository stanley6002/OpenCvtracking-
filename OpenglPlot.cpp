//
//  OpenglPlot.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/24/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "OpenglPlot.h"
#include <iostream>


const float DEG2RAD = 3.141593f / 180;
int GlobalzoomFact=  110;

Matrix4X4 matrixModelView;    // = matrixView * matrixModel
Matrix4X4 matrixProjection;


float Max_VX = -9999;
float Min_Vx = 9999;
float Max_VY = -9999;
float Min_VY = 9999;
float Max_VZ = -9999;
float Min_VZ = 9999;

int inx=0;


void OpenGLPlot::error_callback(int error, const char* description)
{
       fputs(description, stderr);
}
/// a global call back function  
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GL_TRUE);
}
void OpenGLPlot::Initialized ()
{
        if (!glfwInit())
              exit(EXIT_FAILURE);
        window= glfwCreateWindow(this-> width , this ->height, "OpenGL1", NULL, NULL);
        //window2= glfwCreateWindow(this-> width , this ->height, "OpenGL2", NULL, NULL);
        if (!window)
           {
              glfwTerminate();
              exit(EXIT_FAILURE);
           }

         //glfwMakeContextCurrent(window);
         //glfwMakeContextCurrent(window2);
         glfwSwapInterval(1);
         //glfwSetKeyCallback (window, key_callback);
}
void OpenGLPlot:: Setview(const vector<v3_t> V3Dpts)
{
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
            
            vector<v3_t> _3Dpts;
            if( (int) V3Dpts.size()<=2001)
            {
               vector<v3_t> temp3D(V3Dpts.begin(), V3Dpts.begin()+(int)V3Dpts.size()); 
               _3Dpts.swap (temp3D);
         
            }
            else
            {
                int idx = (int) V3Dpts.size()-2000;
                //vector<v3_t> temp3D;
                //for(int i=0;i<500;i++)
                //{
                // temp3D.push_back(V3Dpts[(int) V3Dpts.size()-i]);
                //
               //}
                vector<v3_t> temp3D(V3Dpts.end()-2000, V3Dpts.end());
               
                _3Dpts.swap(temp3D);
            }
            cout<<"current size "<< V3Dpts.size();
            
            ReadObject(_3Dpts);
            glfwMakeContextCurrent(window);
        
            glLineWidth(4);
            glfwGetFramebufferSize(window, &this->width, &this->height);
            
            glMatrixMode(GL_MODELVIEW);
            Matrix4X4 matrixModelView2; 
            glLoadIdentity();
            matrixModelView2.identity();
            if( Min_VZ>  0)
            {
                Min_VZ= -Min_VZ;
            }
            matrixModelView2.translate(-0.0 ,0.0, Min_VZ-20);
            cout<< "center "<<center_x<<" "<<center_y<<endl;
            glLoadMatrixf(matrixModelView2.getTranspose());
        
            glPushMatrix();
            glLoadIdentity();
            glMatrixMode(GL_MODELVIEW);

            PlotVertex((int) _3Dpts.size() , _3Dpts);
  
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glViewport(0, 0, width, height);
            Reshape(this->width, this-> height);

            glfwSwapBuffers(window);
            glfwPollEvents();      
     
}
void OpenGLPlot::SetupFrustrum()
{
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glFrustum (-8.0, 8.0, -5.5, 5.5, 1.0 , 45.5);    
    glMatrixMode (GL_MODELVIEW);
    glPushMatrix();
}

Matrix4X4 setFrustum(float l, float r, float b, float t, float n, float f)
{
    Matrix4X4 mat;
    mat[0]  =  2 * n / (r - l);
    mat[2]  =  (r + l) / (r - l);
    mat[5]  =  2 * n / (t - b);
    mat[6]  =  (t + b) / (t - b);
    mat[10] = -(f + n) / (f - n);
    mat[11] = -(2 * f * n) / (f - n);
    mat[14] = -1;
    mat[15] =  0;
    return mat;
}
Matrix4X4 setFrustum(float fovY, float aspectRatio, float front, float back)
{
    float tangent = tanf(fovY/2 * DEG2RAD);   // tangent of half fovY
    float height = front * tangent;           // half height of near plane
    float width = height * aspectRatio;       // half width of near plane 
    // params: left, right, bottom, top, near, far
    return setFrustum(-width, width, -height, height, front, back);
}
void OpenGLPlot::Reshape(int height, int width )
{   
    glViewport(0, 0, height, width);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    cout<<"theta " <<theta<<endl;
    matrixProjection = setFrustum(80 ,(float)width/height ,  0.1 , 700);    // given near and far from point set
    glLoadMatrixf(matrixProjection.getTranspose());
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
void OpenGLPlot::PlotCamera(double *t_relative)
{
    glPushMatrix();
    glTranslated(-0.0 ,-0.0  , -30.0); 
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f,0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0, 0, 1); 
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();
    
    glTranslated(t_relative[0] ,(-1)* t_relative[1]  ,t_relative[2]); 
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f,0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0, 0, 1); 
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();
    glPushMatrix();
    glfwSwapBuffers(window);
    //glClear(GL_COLOR_BUFFER_BIT);
   
    
}
 void OpenGLPlot::PlotVertex(int size_, const vector<v3_t> V3Dpts)
{

    glLoadIdentity();
    glPointSize(2.0f);
    //Matrix4X4 matrixModelView2; 
    //glLoadIdentity();
    //matrixModelView2.identity();
    //matrixModelView2.rotate(90, 1.0f, 0.0f, 0.0f);
    //matrixModelView2.translate( -0.0, -6.0, -20);
    //glLoadMatrixf(matrixModelView2.getTranspose());
    glPushMatrix();
    //glScaled(0.6,0.6,0.6);
    glTranslated(-center_x, +center_y  , center_z-10);
    glBegin( GL_POINTS);
    for (int i=0;i<size_;i++)
    {
        glColor3f(1, 1, 1); 
        glVertex3f(V3Dpts[i].p[0],(-1)*V3Dpts[i].p[1] ,V3Dpts[i].p[2]);
        //cout<< V3Dpts[i].p[0]<<" "<<V3Dpts[i].p[1]<<" "<<V3Dpts[i].p[2]<<endl;
    }
    glEnd();
    glPushMatrix();


    glDisable(GL_DEPTH_TEST);
    //glfwSwapBuffers(window);
    //glfwPollEvents();

}
void OpenGLPlot::PlotVertex2(int size_, const vector<v3_t> V3Dpts)
{

    glBegin(GL_TRIANGLE_FAN);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0.5, 0);
    
    glColor3f(1, 1, 0);
    glVertex3f(-0.5, -0.5, 0.5);
    
    glColor3f(1, 1, 1);
    glVertex3f(0.5, -0.5, 0.5);
    
    glColor3f(0, 1, 1);
    glVertex3f(0.5, -0.5, -0.5);
    
    glColor3f(0, 0, 1);
    glVertex3f(-0.5, -0.5, -0.5);
    
    glColor3f(0, 1, 0);
    glVertex3f(-0.5, -0.5, 0.5);
    glEnd;

    glPushMatrix();
    
}
void OpenGLPlot::ReadObject(const vector<v3_t> V3Dpts)
{
    
         Max_VX = -9999;
         Min_Vx = 9999;
         Max_VY = -9999;
         Min_VY = 9999;
         Max_VZ = -9999;
         Min_VZ = 9999;
        int i;

        int NumTris= (int)V3Dpts.size();

    for (i=0; i<NumTris; i++) // read triangles
      {
       
        if (V3Dpts[i].p[0]>(Max_VX))
                Max_VX=  V3Dpts[i].p[0];
        if (V3Dpts[i].p[0]<(Min_Vx))    
                Min_Vx= V3Dpts[i].p[0];
        
        if (V3Dpts[i].p[1]>(Max_VY))
                    Max_VY= V3Dpts[i].p[1];
        if (V3Dpts[i].p[1]<(Min_VY))    
                    Min_VY= V3Dpts[i].p[1];
        if (V3Dpts[i].p[2]>(Max_VZ))
                    Max_VZ= V3Dpts[i].p[2];
        if (V3Dpts[i].p[2]<(Min_VZ))    
                    Min_VZ= V3Dpts[i].p[2];
    
      }
         center_x= (Min_Vx+Max_VX)/2.0f;
         center_y= (Min_VY+Max_VY)/2.0f;
         center_z= (Min_VZ+Max_VZ)/2.0f;

        float   radius= sqrt((Max_VX - center_x)*(Max_VX -center_x) + (Max_VY - center_y)*(Max_VY - center_y) + (Max_VZ- center_z)*(Max_VZ- center_z));
         f_Distance = radius/1.53;
        float _Distance=radius/2.0;
        if (Max_VZ<0)
        {
            Max_VZ=-Max_VZ;
        }

        double  D_Max_VZ= double(Max_VZ);
        _Distance = _Distance-D_Max_VZ;
        //if (Distance<0)
        //{
        //    Distance= -Distance;
        //}
        double  D_Max_VY= double((Max_VY-Min_VY)/2);
         theta = 120*(atan2(D_Max_VY,_Distance))/M_pi;

        Dnear = f_Distance - radius;
        Dfar  = _Distance ;


}
OpenGLPlot:: OpenGLPlot (int width, int height)
 {
         OpenGLPlot:: height = height;
         OpenGLPlot:: width  = width;            
         OpenGLPlot::Initialized();
 }
OpenGLPlot::~OpenGLPlot()
{
}