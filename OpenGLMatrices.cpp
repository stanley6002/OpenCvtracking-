
#include <cmath>
#include <algorithm>
#include "OpenGLMatrices.h"

const float DEG2RAD = 3.141593f / 180;


Matrix4X4& Matrix4X4::transpose()
{
    std::swap(ele[1],  ele[4]);
    std::swap(ele[2],  ele[8]);
    std::swap(ele[3],  ele[12]);
    std::swap(ele[6],  ele[9]);
    std::swap(ele[7],  ele[13]);
    std::swap(ele[11], ele[14]);

    return *this;
}


Matrix4X4& Matrix4X4::translate(float x, float y, float z)
{
    ele[0] += ele[12]*x;   ele[1] += ele[13]*x;   ele[2] += ele[14]*x;   ele[3] += ele[15]*x;
    ele[4] += ele[12]*y;   ele[5] += ele[13]*y;   ele[6] += ele[14]*y;   ele[7] += ele[15]*y;
    ele[8] += ele[12]*z;   ele[9] += ele[13]*z;   ele[10]+= ele[14]*z;   ele[11]+= ele[15]*z;
    return *this;
}
Matrix4X4& Matrix4X4::rotate(float angle, float x, float y, float z)
{
    float c = cosf(angle * DEG2RAD);    // cosine
    float s = sinf(angle * DEG2RAD);    // sine
    float xx = x * x;
    float xy = x * y;
    float xz = x * z;
    float yy = y * y;
    float yz = y * z;
    float zz = z * z;

    // build rotation matrix
    Matrix4X4 mat;
mat[0] = xx * (1 - c) + c;             mat[1] = xy * (1 - c) - z * s;
    mat[2] = xz * (1 - c) + y * s;         mat[3] = 0;
    mat[4] = xy * (1 - c) + z * s;         mat[5] = yy * (1 - c) + c;
    mat[6] = yz * (1 - c) - x * s;         mat[7] = 0;
    mat[8] = xz * (1 - c) - y * s;         mat[9] = yz * (1 - c) + x * s;
    mat[10]= zz * (1 - c) + c;             mat[11]= 0;
    mat[12]= 0;                            mat[13]= 0;
    mat[14]= 0;                            mat[15]= 1;
   
    *this = mat * (*this);

    return *this;
}
Matrix4X4::Matrix4X4()
{
    // initially identity matrix
    identity();
}


Matrix4X4::Matrix4X4(float xx, float xy, float xz, float xw,
                        float yx, float yy, float yz, float yw,
                        float zx, float zy, float zz, float zw,
                        float wx, float wy, float wz, float ww)
{
    //set(xx, xy, xz, xw,  yx, yy, yz, yw,  zx, zy, zz, zw,  wx, wy, wz, ww);
    ele[0] = xx;  ele[1] = xy;  ele[2] = xz;  ele[3] = xw;
    ele[4] = yx;  ele[5] = yy;  ele[6] = yz;  ele[7] = yw;
    ele[8] = zx;  ele[9] = zy;  ele[10]= zz;  ele[11]= zw;
    ele[12]= wx;  ele[13]= wy;  ele[14]= wz;  ele[15]= ww;
}



const float* Matrix4X4::getTranspose()
{
    Transpose[0] = ele[0];   Transpose[1] = ele[4];   Transpose[2] = ele[8];   Transpose[3] = ele[12];
    Transpose[4] = ele[1];   Transpose[5] = ele[5];   Transpose[6] = ele[9];   Transpose[7] = ele[13];
    Transpose[8] = ele[2];   Transpose[9] = ele[6];   Transpose[10]= ele[10];  Transpose[11]= ele[14];
    Transpose[12]= ele[3];   Transpose[13]= ele[7];   Transpose[14]= ele[11];  Transpose[15]= ele[15];
    return Transpose;
}



Matrix4X4& Matrix4X4::identity()
{
    ele[0] = ele[5] = ele[10] = ele[15] = 1.0f;
    ele[1] = ele[2] = ele[3] = ele[4] = ele[6] =ele[7] = ele[8] = ele[9] = ele[11] = ele[12] = ele[13] = ele[14] = 0.0f;
    return *this;
}

 Matrix4X4 Matrix4X4::operator*( Matrix4X4& n) const
{
    return Matrix4X4(ele[0]*n[0]  + ele[1]*n[4]  + ele[2]*n[8]  + ele[3]*n[12],   ele[0]*n[1]  + ele[1]*n[5]  + ele[2]*n[9]  + ele[3]*n[13],   ele[0]*n[2]  +         ele[1]*n[6]  + ele[2]*n[10]  + ele[3]*n[14],   ele[0]*n[3]  + ele[1]*n[7]  + ele[2]*n[11]  + ele[3]*n[15],
                   ele[4]*n[0]  + ele[5]*n[4]  + ele[6]*n[8]  + ele[7]*n[12],   ele[4]*n[1]  + ele[5]*n[5]  + ele[6]*n[9]  + ele[7]*n[13],   ele[4]*n[2]  + ele[5]*n[6]  + ele[6]*n[10]  + ele[7]*n[14],   ele[4]*n[3]  + ele[5]*n[7]  + ele[6]*n[11]  + ele[7]*n[15],
                   ele[8]*n[0]  + ele[9]*n[4]  + ele[10]*n[8] + ele[11]*n[12],  ele[8]*n[1]  + ele[9]*n[5]  + ele[10]*n[9] + ele[11]*n[13],  ele[8]*n[2]  + ele[9]*n[6]  + ele[10]*n[10] + ele[11]*n[14],  ele[8]*n[3]  + ele[9]*n[7]  + ele[10]*n[11] + ele[11]*n[15],
                   ele[12]*n[0] + ele[13]*n[4] + ele[14]*n[8] + ele[15]*n[12],  ele[12]*n[1] + ele[13]*n[5] + ele[14]*n[9] + ele[15]*n[13],  ele[12]*n[2] + ele[13]*n[6] + ele[14]*n[10] + ele[15]*n[14],  ele[12]*n[3] + ele[13]*n[7] + ele[14]*n[11] + ele[15]*n[15]);
}


 float Matrix4X4::operator[](int index) const
{
    return ele[index];
}

 float& Matrix4X4::operator[](int index)
{
    return ele[index];
}

Matrix4X4 operator*(float s, const Matrix4X4& rhs)
{
    return Matrix4X4(s*rhs[0], s*rhs[1], s*rhs[2], s*rhs[3], s*rhs[4], s*rhs[5], s*rhs[6], s*rhs[7], s*rhs[8], s*rhs[9], s*rhs[10], s*rhs[11], s*rhs[12], s*rhs[13], s*rhs[14], s*rhs[15]);
}

