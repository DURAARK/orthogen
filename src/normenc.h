#ifndef _NORMALENC_H_
#define _NORMALENC_H_

#include <cmath>
#include "types.h"

namespace OrthoGen {

// adaption of the source code of [Schinko11]:
// "Simple and Efficient Normal Encoding with Error Bounds"
// Schinko, Christoph; Ullrich, Torsten; Fellner, Dieter W.
// DOI = {10.2312/LocalChapterEvents/TPCG/TPCG11/063-065}

class NormalEncoding {
private:
  int resolution;
  int sampling;
  int maximum;

public:

  NormalEncoding(int bits) : resolution(bits) 
  {
    sampling = (int)std::floor(std::sqrt(std::pow(2.0, resolution) / 6));
    maximum = 6 * sampling * sampling;
  }

  int getMaximum() const { return maximum; }

  Vec3d i2n(int i) {
    const int i2uv_t1 = i / 6;
    const int i2uv_t2 = i2uv_t1 / sampling;
    const double i2uv_t3 = 1.0 / sampling;
    const int i2uv_t7 = i2uv_t1 % sampling;
    const int i2uv_t11 = i % 6;
    const double u =
        -0.10e1 + 0.20e1 * (double)i2uv_t2 * (double)i2uv_t3 + (double)i2uv_t3;
    const double v =
        -0.10e1 + 0.20e1 * (double)i2uv_t7 * (double)i2uv_t3 + (double)i2uv_t3;
    const int w = i2uv_t11;
    const double tanUV_t3 = std::tan(M_PI * u / 4.0);
    const double tanUV_t4 = tanUV_t3 * tanUV_t3;
    const double tanUV_t7 = std::tan(M_PI * v / 4.0);
    const double tanUV_t8 = tanUV_t7 * tanUV_t7;
    const double tanUV_t10 = std::sqrt(1.0 + tanUV_t4 + tanUV_t8);
    const double tanUV_t11 = 1.0 / tanUV_t10;
    const double tanUV_t12 = tanUV_t3 * tanUV_t11;
    const double tanUV_t13 = tanUV_t7 * tanUV_t11;
    double x, y, z;
    switch (w) {
    case 0:
      x = -tanUV_t12; y = -tanUV_t13; z = tanUV_t11;
      break;
    case 1:
      x = -tanUV_t12; y = -tanUV_t13; z = -tanUV_t11;
      break;
    case 2:
      x = tanUV_t11; y = -tanUV_t12; z = -tanUV_t13;
      break;
    case 3:
      x = -tanUV_t11; y = -tanUV_t12; z = -tanUV_t13;
      break;
    case 4:
      x = -tanUV_t12; y = tanUV_t11; z = -tanUV_t13;
      break;
    case 5:
      x = -tanUV_t12; y = -tanUV_t11; z = -tanUV_t13;
      break;
    default:
      x = 0; y = 0; z = 0;
    }
    return Vec3d(x, y, z);
  }

  int n2i(double x, double y, double z) 
  {
    const double nx0x = 0, nx0y = 1, nx0z = -1;
    const double nx1x = 0, nx1y = -1, nx1z = -1;
    const double ny0x = 1, ny0y = 0, ny0z = -1;
    const double ny1x = -1, ny1y = 0, ny1z = -1;
    const double nz0x = 1, nz0y = -1, nz0z = 0;
    const double nz1x = 1, nz1y = 1, nz1z = 0;
    const bool testx0 = x * nx0x + y * nx0y + z * nx0z > 0;
    const bool testx1 = x * nx1x + y * nx1y + z * nx1z > 0;
    const bool testy0 = x * ny0x + y * ny0y + z * ny0z > 0;
    const bool testy1 = x * ny1x + y * ny1y + z * ny1z > 0;
    const bool testz0 = x * nz0x + y * nz0y + z * nz0z > 0;
    const bool testz1 = x * nz1x + y * nz1y + z * nz1z > 0;
    
    int side;
    
    if      (testx0 && testx1 && testy0 && testy1)     { side = 1; } 
    else if (!testx0 && !testx1 && !testy0 && !testy1) { side = 0; } 
    else if (testy0 && !testy1 && testz0 && testz1)    { side = 2; } 
    else if (!testy0 && testy1 && !testz0 && !testz1)  { side = 3; } 
    else if (!testx0 && testx1 && testz0 && !testz1)   { side = 5; } 
    else                                               { side = 4; }

    double scale;
    switch (side) 
    {
        case 0: scale = 1 / z;  break;
        case 1: scale = -1 / z; break;
        case 2: scale = 1 / x;  break;
        case 3: scale = -1 / x; break;
        case 4: scale = 1 / y;  break;
        case 5: scale = -1 / y; break;
        default: scale = 0; 
    }
    const double sx = scale * x, sy = scale * y, sz = scale * z;
    double u, v;
    switch (side) {
    case 0: case 1:
        u = (-4 / M_PI) * std::atan(sx);
        v = (-4 / M_PI) * std::atan(sy);
        break;
    case 2: case 3:
        u = (-4 / M_PI) * std::atan(sy);
        v = (-4 / M_PI) * std::atan(sz);
        break;
    case 4: case 5:
        u = (-4 / M_PI) * std::atan(sx);
        v = (-4 / M_PI) * std::atan(sz);
        break;
    default:
      u = 0;
      v = 0;
    }

    const int p = std::max( 0, std::min(sampling - 1,
                          (int)std::round(0.5 * (sampling - 1 + u * sampling))));
    const int q = std::max( 0, std::min(sampling - 1,
                          (int)std::round(0.5 * (sampling - 1 + v * sampling))));
    return (p * sampling + q) * 6 + side;
  }
};
}
#endif
