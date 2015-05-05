//
//  Utility.cpp
//  Opengl_assigment 
//
//  Created by chih-hsiang chang on 9/23/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#include "Utility.h"
//#include <vector.h>
# include <math.h>
//extern vector<Vertices> _3D_vector;
static int tri_size=0;
//= new Vertices[];
//extern GLuint list= glGenLists(2);
//Vertices  *Tris;
float x_R= 1.0f;
float x_G= 0.0f;
float x_B= 0.0f;
float center_x;
float center_y;
float center_z;
float radius;
float f_Distance;
float Dnear;
float Dfar;
double theta;
#define M_pi 3.1416
GLuint main_(char *FileName,int x)
{
     GLuint list_main;
     list_main  = glGenLists (1);
    //glNewList(list_main, GL_COMPILE);
    //glColor3f(1.0f, 0.0f, 0.0f);
    //glBegin (GL_TRIANGLES);
    //glVertex2f (0.0, 0.0);
    //glVertex2f (1.0, 1.0);
    //glVertex2f (0.0, 1.0);
    //glEnd ();  
    //glEndList();
   
    //ptr= (Vertices*)ReadObject(FileName);
    Vertices *ptr= ReadObject(FileName);
    list_main=createDL(ptr, x);
  
    //list_main=glGenLists(1);
    //cout<<center_x<<" "<<center_y<<" "<<center_z<<endl;
    return(list_main);
    
     //cout<<"test"<<endl;
}

camera_location* ReadCamera(char* FileName, int& NumTris)
{
    FILE* fp = fopen(FileName,"r"); 
    test(fp);
    char ch;
    if (fp==NULL) { printf("ERROR: unable to open TriObj [%s]!\n",FileName); exit(1); }
    fscanf(fp, "%c", &ch); 
    while(ch!= '\n')
        // skip the first line – object’s name
    fscanf(fp, "%c", &ch);
    fscanf(fp,"# triangles = %d\n", &NumTris); 
    printf ("Reading in %s (%d triangles). . .\n", FileName, NumTris);
    camera_location  * position= new camera_location[NumTris];
    for (int i=0; i<NumTris; i++) // read triangles
    {
        fscanf(fp, "%f %f %f\n", 
               &(position[i].x), &(position[i].y), &(position[i].z));
    }
     fclose(fp);
    return(position);
}

Vertices *ReadObject(char *FileName)
{   
    float Max_VX = -9999;
    float Min_Vx = 9999;
    float Max_VY = -9999;
    float Min_VY = 9999;
    float Max_VZ = -9999;
    float Min_VZ = 9999;
    int i;
    int material_count;
    int NumTris;
    char ch;
    FILE* fp = fopen(FileName,"r"); 
    
    test(fp);
    if (fp==NULL) { printf("ERROR: unable to open TriObj [%s]!\n",FileName); exit(1); }
    fscanf(fp, "%c", &ch); 
    while(ch!= '\n')
    // skip the first line – object’s name
    fscanf(fp, "%c", &ch);
    fscanf(fp,"# triangles = %d\n", &NumTris); 
    float x1;
    float x2;
    float x3;
    //fscanf(fp,"Material count = %d\n", &material_count); // read material count
    //float color_index[3];
    //float shine [material_count];
    //ve_ele *ambient = new ve_ele[material_count];
    //ve_ele *diffuse = new ve_ele[material_count];
    //ve_ele *specular = new ve_ele[material_count];
    // read # of triangles 
    //fscanf(fp,"Material count = %d\n", &material_count); // read material count
    //for (i=0; i<material_count; i++)
    //{ 
    //    fscanf(fp, "ambient color %f %f %f\n", &(ambient[i].x), &(ambient[i].y), &(ambient[i].z)); 
    //    fscanf(fp, "diffuse color %f %f %f\n", &(diffuse[i].x), &(diffuse[i].y), &(diffuse[i].z)); 
    //    fscanf(fp, "specular color %f %f %f\n", &(specular[i].x), &(specular[i].y), &(specular[i].z)); 
    //    fscanf(fp, "material shine %f\n", &(shine[i]));
    //}
   // fscanf(fp, "%c", &ch); 
   // while(ch!= '\n') // skip documentation line 
   // fscanf(fp, "%c", &ch);
    printf ("Reading in %s (%d triangles). . .\n", FileName, NumTris);
    Vertices * Tris= new Vertices[NumTris];
    tri_size= NumTris;
  for (i=0; i<NumTris; i++) // read triangles
   {
//    fscanf(fp, "%f %f %f %f %f %f\n", 
//               &(Tris[i].v0), &(Tris[i].v1), &(Tris[i].v2), 
//               &(Tris[i].Color[0]),&(Tris[i].Color[1]), &(Tris[i].Color[2]));
           fscanf(fp, "%f %f %f %f %f %f\n", 
                      &(Tris[i].v0), &(Tris[i].v1), &(Tris[i].v2),&(Tris[i].Color[0]),&(Tris[i].Color[1]), &(Tris[i].Color[2]), &x1,&x2,&x3);
   if (Tris[i].v0>(Max_VX))
       Max_VX= Tris[i].v0;
   if (Tris[i].v0<(Min_Vx))    
       Min_Vx= Tris[i].v0;
   if (Tris[i].v1>(Max_VY))
        Max_VY= Tris[i].v1;
   if (Tris[i].v1<(Min_VY))    
        Min_VY= Tris[i].v1;
   if (Tris[i].v2>(Max_VZ))
           Max_VZ= Tris[i].v2;
  if (Tris[i].v2<(Min_VZ))    
           Min_VZ= Tris[i].v2;
   }
    fclose(fp);
    center_x= (Min_Vx+Max_VX)/2.0f;
    center_y= (Min_VY+Max_VY)/2.0f;
    center_z= (Min_VZ+Max_VZ)/2.0f;
    //center_z=20;
    radius= sqrt((Max_VX - center_x)*(Max_VX -center_x) + (Max_VY - center_y)*(Max_VY - center_y) + (Max_VZ- center_z)*(Max_VZ- center_z));
    f_Distance = radius/1.53;
    float _Distance=radius/2.0;
    if (Max_VZ<0)
    {
        Max_VZ=-Max_VZ;
    }
   
    double  D_Max_VZ= double(Max_VZ);
    double distance =_Distance-D_Max_VZ;
    if (distance<0)
    {
        distance=-distance;
    }
    double  D_Max_VY= double((Max_VY-Min_VY)/2);
   double theta1 = 120*(2.0*atan2(D_Max_VY,_Distance))/M_pi;
    theta =theta1;
    Dnear = f_Distance - radius;
    Dfar  = f_Distance + radius;
    cout<<"near "<<Dnear<<" "<<Dfar<<endl;
    //Dnear =180;
    //Dfar=0.9;
    return(Tris);

}
void test(FILE *data)
{
    cout<<"reseived"<<endl;

}
GLuint createDL(Vertices *Tris, int x)
{
     GLuint list=glGenLists(1);
    if(!list) return list;  // failed to create a list, return 0
    glNewList(list, GL_COMPILE);
    //glPolygonMode(GL_FRONT, GL_LINE);
    //glPolygonMode(GL_BACK, GL_LINE);
    switch (x) 
    {
        case 0:
            glPointSize(2.0);
            glBegin(GL_POINTS);
           
            break;
        case 1:
           glBegin(GL_TRIANGLES);
            break; 
        case 2:
            glBegin(GL_POLYGON);
            break;   
        default:
            break;
     } 
    glBegin(GL_POINTS);
    for (int i=0;i<tri_size;i++)
    {
  //  glNormal3f(-1.0f, -1.0f, 1.0f); //Specify the first normal
    glVertex3f(Tris[i].v0,Tris[i].v1,Tris[i].v2);

    glColor3f((Tris[i].Color[0]/255),(Tris[i].Color[1]/255),(Tris[i].Color[2]/255));  
 //   glNormal3f(0.0f, 1.0f, 1.0f); //Specify the second normal
 //   glVertex3f(Tris[i].v1.x, Tris[i].v1.y,Tris[i].v1.z);
 //   glNormal3f(1.0f, -1.0f, 1.0f); //Specifythe third normal
 //   glVertex3f(Tris[i].v2.x, Tris[i].v2.y,Tris[i].v2.z);
    }
    glEnd();
   // glBegin(GL_QUADS);
    
//    glVertex3f(-0.203849 ,-0.033,-0.053138);
//    glVertex3f(0.203849  ,-0.033,-0.053138);
//     glVertex3f(0.203849  ,-0.033,0.069);
//    glVertex3f(-0.203849 ,-0.033,0.069);
   
   
   //  glEnd();
       glEndList();
//    Index++;
//    glNewList(list+Index, GL_COMPILE);
//    glColor3f(1.0f, 1.0f, 1.0f);
    return (list);
}

camera_location* ReadCamera_center (char* FileName, int& NumTris)
{
    FILE* fp = fopen(FileName,"r"); 
    test(fp);
    char ch;
    if (fp==NULL) { printf("ERROR: unable to open TriObj [%s]!\n",FileName); exit(1); }
    //fscanf(fp, "%c", &ch); 
    // while(ch!= '\n')
        // skip the first line – object’s name
    //     fscanf(fp, "%c", &ch);
    
    fscanf(fp,"# triangles = %d\n", &NumTris); 
    printf ("Reading in %s (%d triangles). . .\n", FileName, NumTris);
    camera_location  * position= new camera_location[2*NumTris];
    for (int i=0; i<2*NumTris; i++) // read triangles
    {
        fscanf(fp, "%f %f %f\n", 
               &(position[i].x), &(position[i].y), &(position[i].z));
        //fscanf(fp, "%f %f %f\n", 
        //       &(position[i].x), &(position[i].y), &(position[i].z));
    }
    fclose(fp);

    return(position);
}
