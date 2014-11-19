#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <vector>
#include <cassert>
#include "jpeglib.h"

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


    template <class IMGTYPE>
    bool loadJPEG(const char *fname, IMGTYPE &img)
    {
        struct jpeg_decompress_struct cinfo;
        struct jpeg_error_mgr jerr;

        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_decompress(&cinfo);

        FILE *infile = fopen(fname, "rb");

        if (infile == NULL || ferror(infile))
        {
            fprintf(stderr, "error opening file %s for reading.\n", fname);
            return false;
        }
        jpeg_stdio_src(&cinfo, infile);
        jpeg_read_header(&cinfo, TRUE);

        // assume RGB
        img.resize(cinfo.image_width, cinfo.image_height);
        jpeg_start_decompress(&cinfo);

        // create vector with scanline start ptrs
        std::vector<JSAMPROW> rowptr(cinfo.image_height);
        for (unsigned int i = 0; i<cinfo.image_height; ++i)
        {
            rowptr[i] = (&img(0, i)); //     &m_data[i * cinfo.image_width * m_channels]
        }

        // read scanlines
        while (cinfo.output_scanline < cinfo.output_height)
        {
            jpeg_read_scanlines(&cinfo, &rowptr[cinfo.output_scanline], 10);
        }

        jpeg_finish_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        fclose(infile);

        return true;
    }

    template <class IMGTYPE>
    bool saveJPEG(const char *fname, const IMGTYPE &img, const int quality=80)
    {
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        FILE * outfile;
        JSAMPROW row_pointer[1];
        int row_stride;

        assert(img.bpp() == 24);

        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);

        if ((outfile = fopen(fname, "wb")) == NULL) {
            fprintf(stderr, "error opening file %s for writing\n", fname);
            return false;
        }
        jpeg_stdio_dest(&cinfo, outfile);

        cinfo.image_width = img.width();
        cinfo.image_height = img.height();
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_RGB;

        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, quality, TRUE);
        jpeg_start_compress(&cinfo, TRUE);

        row_stride = img.width() * 3;
        while (cinfo.next_scanline < cinfo.image_height) {
            row_pointer[0] = (JSAMPROW) &img.getData()[cinfo.next_scanline * row_stride];
            (void)jpeg_write_scanlines(&cinfo, row_pointer, 1);
        }

        jpeg_finish_compress(&cinfo);
        fclose(outfile);
        jpeg_destroy_compress(&cinfo);
        return true;
    }

#endif
