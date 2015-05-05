///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

#ifndef MATH_MATRICES_H
#define MATH_MATRICES_H



///////////////////////////////////////////////////////////////////////////
// 4x4 matrix
///////////////////////////////////////////////////////////////////////////
class Matrix4X4
{
public:
    // constructors
    Matrix4X4();  // init with identity
    Matrix4X4(float xx, float xy, float xz, float xw,
            float yx, float yy, float yz, float yw,
            float zx, float zy, float zz, float zw,
            float wx, float wy, float wz, float ww);
    const float* getTranspose();                        // return transposed matrix
    Matrix4X4&    identity();
    Matrix4X4&    transpose();                            // transpose itself and return reference
    Matrix4X4&    translate(float x, float y, float z);   // translation by (x,y,z)
    Matrix4X4&    rotate(float angle, float x, float y, float z);
    Matrix4X4     operator*( Matrix4X4& rhs) const;    // multiplication: M3 = M1 * M2
    float       operator[](int index) const;           
    float&      operator[](int index);                 
protected:

private:
    float ele[16];
    float Transpose[16];                                       // transpose m

};

#endif
