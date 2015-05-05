//
//  Utility.h
//  Opengl_assigment 
//
//  Created by chih-hsiang chang on 9/23/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

using namespace std;
#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <vector.h>

extern float center_x;
extern float center_y;
extern float center_z;
extern float radius;
extern float f_Distance;
extern float Dnear;
extern float Dfar;
extern float x_R;
extern float x_G;
extern float x_B;
extern double theta;

typedef struct
{ 
    float x;
    float y;
    float z;
} ve_ele;

typedef struct
{
    float v0;
    float v1;
    float v2;
    float Color[3];
} Vertices; 
typedef struct 
{
    float x;
    float y;
    float z;
}camera_location;

GLuint main_(char *FileName,int x);
Vertices *ReadObject(char *FileName);
GLuint createDL(Vertices *Tris,int x);
void test(FILE *TEST);
camera_location* ReadCamera(char* FileName, int& NumTris);
camera_location* ReadCamera_center (char* FileName, int& NumTris);