#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <cmath>
#include "matrix.h"

// header only library
template <class T>
class Camera
{
typedef MathVector<T,3> Vec3;
// A simple Camera utility class
//
// transformation from camera to world coordinates:
//
// [wx]          [x_x]      [y_x]      [z_x]             [cx]
// [wy] = O + cx*[x_y] + cy*[y_y] + cz*[z_y] = [X|Y|Z] * [cy] + O
// [wz]          [x_z]      [y_z]      [z_z]             [cz]
//                X        Y        Z          A
//
// with O being the camera origin and [x][y][z] being three
// normalized orthogonal axis directions in world coordinates.
// Therefore, the transformation matrix A consists of the three
// axis vectors as columns.

public:
struct ViewFrustum
{
  Vec3 min;
  Vec3 max;
};

private:
    // camera coordinate system
    Vec3 O;          // origin
    Vec3 X;          // right
    Vec3 Y;          // view
    Vec3 Z;          // up

    // view parameters
    T m_zNear, m_zFar, m_fov;

 public:
    Camera() :
    O(0,0,10), X(1,0,0), Y(0,1,0), Z(0,0,1)
    {
        setupFrustum(0.01f, 100.0f, 45.0f);
    }

    inline Vec3 getPosition() const { return O; }
    inline Vec3 getX()        const { return X; }
    inline Vec3 getY()        const { return Y; }
    inline Vec3 getZ()        const { return Z; }
    inline T    getFOV()      const { return m_fov; }

    ViewFrustum getViewFrustum(const int w, const int h) const
    {
       ViewFrustum f;
       T aspect;
       f.min[2] = m_zNear;
       f.max[2] = m_zFar;
       if (w >= h) {
          aspect = (T)w/(T)h;
          f.max[1] = m_zNear * tan(m_fov * (T)3.141592653589793 / (T)360.0);
          f.min[1] = -f.max[1];
          f.min[0] = f.min[1] * aspect;
          f.max[0] = f.max[1] * aspect;
       } else {
          aspect = (T)h/(T)w;
          f.max[0] = m_zNear * tan(m_fov * (T)3.141592653589793 / (T)360.0);
          f.min[0] = -f.max[0];
          f.min[1] = f.min[0] * aspect;
          f.max[1] = f.max[0] * aspect;
       }
       return f;
    }

    ViewFrustum getViewFrustumWorld(const int w, const int h)
    {
        ViewFrustum local = getViewFrustum(w, h);
        Matrix<T, 4> T = cam2world();
        ViewFrustum world;        
        world.min = T * local.min;
        world.max = T * local.max;
        return world;
    }

    void setupFrustum(const T znear, const T zfar, const T fov)
    {
        m_zNear=znear;
        m_zFar=zfar;
        m_fov=fov;
    }

    void changeFOV(const T fov)
    {
        m_fov = fov;
    }

    // camera -> world coordinates
    Matrix<T,4> cam2world() const
    {
        Matrix<T,4> mv;
        mv(0)=X[0]; mv(4)=Y[0]; mv(8) =Z[0]; mv(12)=O[0];
        mv(1)=X[1]; mv(5)=Y[1]; mv(9) =Z[1]; mv(13)=O[1];
        mv(2)=X[2]; mv(6)=Y[2]; mv(10)=Z[2]; mv(14)=O[2];
        mv(3)=0.0; mv(7)=0.0; mv(11)=0.0; mv(15)=1.0;
        return mv;
    }

    // world -> camera coordinates
    // inverse of
    // wc = O + cx*X + cy*Y + cz*Z = O + [X|Z|Y]*[cc]

    // cc = [X|Y|Z]^-1 * (wc-O)
    // due to orthogonality of XYZ:
    // transformation matrix = [X|Y|Z]^T - ([X|Y|Z]^T*O)
    Matrix<T,4> world2cam() const
    {
        Matrix<T,4> mv;
        mv(0)=X[0]; mv(4)=X[1]; mv(8) =X[2]; mv(12)=-X * O;
        mv(1)=Y[0]; mv(5)=Y[1]; mv(9) =Y[2]; mv(13)=-Y * O;
        mv(2)=Z[0]; mv(6)=Z[1]; mv(10)=Z[2]; mv(14)=-Z * O;
        mv(3)=0.0; mv(7)=0.0; mv(11)=0.0; mv(15)=1.0;
        return mv;
    }

    // movement
    void setPosition(const Vec3 &pos)
    {
      O = pos;
    }

    void setOrientation(const Vec3 &x, const Vec3 &y, const Vec3 &z)
    {
        X = x; Y = y; Z = z;
    }

    template <class TransformationType> 
    void applyRotation(const TransformationType &Trans)
    {
        X = Trans * X;
        Y = Trans * Y;
        Z = Trans * Z;
    }

    void move(const Vec3 &m)
    {
      O += X * m[0] + Y * m[1] + Z * m[2];
    }

    // rotations
    void yaw(const T angle)
    {
      X = RotateVec(X,Y,angle);
      Z = RotateVec(Z,Y,angle);
    }
    void pitch(const T angle)
    {
      Y = RotateVec(Y,X,angle);
      Z = RotateVec(Z,X,angle);
    }
    void roll(const T angle)
    {
      X = RotateVec(X,Z,angle);
      Y = RotateVec(Y,Z,angle);
    }

    void rotateAxis(const Vec3 &axis, const T angle)
    {
        X = RotateVec(X, axis, angle);
        Y = RotateVec(Y, axis, angle);
        Z = RotateVec(Z, axis, angle);
    }

    // std output stream support
    friend std::ostream& operator<<(std::ostream& os, const Camera& c)
    {
       os << "O: " << c.O << " X:" << c.X << " Y:" << c.Y << " Z:" << c.Z << std::endl;
       return os;
    }

};

#endif
