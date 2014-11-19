#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <vector>
#include <cassert>

template <class T>
    struct ImageT
    {
    protected:
        unsigned int W;
        unsigned int H;
        unsigned int BPP;
        std::vector<T> pdata;   // pixel data
    public:
    
        ImageT() : W(0), H(0), BPP(0)
        {
        }

        inline int bufoffset(const int x, const int y) const
        {
            return  (y*W + x) * (BPP / (sizeof(T)* 8));
        }


        inline unsigned int rgb(const int x, const int y) const
        {
           unsigned const int off = bufoffset(x,y);
           assert(off < pdata.size());
           return pdata[off]|(pdata[off+1]<<8)|(pdata[off+2]<<16);
        }

        template <typename VTYPE>
        inline VTYPE rgbT(const int x, const int y) const
        {
            unsigned const int off = bufoffset(x, y);
            assert(off < pdata.size());
            return Vec3d(pdata[off], pdata[off + 1], pdata[off + 2]);
        }

        inline bool isValid()  const { return W != 0 && H != 0 && BPP != 0; }
        inline int width()     const { return W;   }
        inline int height()    const { return H;   }
        inline int bpp()       const { return BPP; }
        inline const T *data() const { return &pdata[0]; }
        inline const std::vector<T>& getData() const { return pdata; }
        inline int buffersize() const { return bufoffset(0,H);  }

        inline std::vector<T>& unsafeData() { return pdata; }

        inline void initialize(int width, int height, int bpp)
        {
            W = width;
            H = height;
            BPP = bpp;
            pdata.resize(buffersize());
        }

        inline void resize(int width, int height, int channels = 3)
        {
           W   = width;
           H   = height;
           BPP = channels * (sizeof(T)*8);
           pdata.resize(buffersize());
        }

        // write access
        inline T &operator()(const int offset) { return pdata[offset]; }
        inline T &operator()(const int x, const int y, const int ch = 0) 
        { 
          return pdata[bufoffset(x,y) + ch]; 
        }

        // read access
        inline const T &operator()(const int x, const int y, const int ch = 0) const 
        { 
            return pdata[bufoffset(x, y) + ch];
        }

        // bilinear interpolation
        template <typename VTYPE>
        inline const VTYPE bilinear(const double x, const double y, int ch = 0) 
            const
        {
            const int l = (int)floor(x); const int r = l >= ((int)W - 1) ? (int)W - 1 : l + 1;
            const int t = (int)floor(y); const int b = t >= ((int)H - 1) ? (int)H - 1 : t + 1;
            VTYPE q11 = rgbT<VTYPE>(l, t), 
                  q21 = rgbT<VTYPE>(r, t), 
                  q12 = rgbT<VTYPE>(l, b), 
                  q22 = rgbT<VTYPE>(r, b);
            return q11*(r - x)*(b - y) +
                   q21*(x - l)*(b - y) +
                   q12*(r - x)*(y - t) +
                   q22*(x - l)*(y - t);
        }

        // clear image
        void clear(T value = 0)
        {
            memset(&pdata[0], value, buffersize());
        }
    };

    typedef ImageT<unsigned char> Image;
    typedef ImageT<float> ImageF;

#endif
