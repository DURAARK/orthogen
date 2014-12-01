#ifndef _TYPES_H_
#define _TYPES_H_

// use eigen matrices
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ifs.h"

// vector and  quaternion types
typedef Eigen::Quaternion<double> Quaterniond;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector2d Vec2d;

typedef Eigen::Matrix< unsigned char, 3, 1 > RGB;
typedef Eigen::Vector4i Vec4i;

// geometry
typedef IFS::IFS< Vec3d, Vec2d > myIFS;

// numerical constants
#define PI 3.141592653589793238462643
#define PIf 3.141592653589793238462643f

#endif
