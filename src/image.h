#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <vector>

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

        inline const unsigned int rgb(const int x, const int y)
        {
           unsigned int offset = (y*W + x) * (BPP/8);
           assert(offset < pdata.size());
           return pdata[offset]|(pdata[offset+1]<<8)|(pdata[offset+2]<<16);
        }

        inline bool isValid()  const { return W != 0 && H != 0 && BPP != 0; }
        inline int width()     const { return W;   }
        inline int height()    const { return H;   }
        inline int bpp()       const { return BPP; }
        inline const T *data() const { return &pdata[0]; }
        inline const std::vector<T>& getData() const { return pdata; }
        inline int buffersize() const {  return W*H*BPP / (sizeof(T)*8);  }

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

        inline T &operator()(int offset) { return pdata[offset]; }

        inline T &operator()(int x, int y, int ch = 0) 
        { 
          return pdata[(y * W + x) * BPP / (sizeof(T)*8) + ch]; 
        }

        inline const T &operator()(int x, int y, int ch = 0) const 
        { 
            return pdata[(y * W + x) * BPP / (sizeof(T)*8) + ch];
        }
        void clear(T value = 0) 
        {
            memset(&pdata[0], value, W*H*BPP/(sizeof(T)*8));
        }
    };

    typedef ImageT<unsigned char> Image;
    typedef ImageT<float> ImageF;

#endif
