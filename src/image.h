#ifndef _IMAGE_H_
#define _IMAGE_H_

// simple image class
// ulrich.krispel@fraunhofer.at

#include <vector>
#include <cassert>
#include <limits>
#include "jpeglib.h"

template <class T> struct ImageT {
  struct AABB2D {
    int l, t, w, h;
    inline AABB2D(int L, int T, int W, int H) : l(L), t(T), w(W), h(H) {}
    inline int r() const { return l + w; }
    inline int b() const { return t + h; }
    inline double x2rel(double x) const { return (x - (double)l) / (double)w; }
    inline double y2rel(double y) const { return (y - (double)t) / (double)h; }
    inline double rel2x(double rx) const { return rx * w + l; }
    inline double rel2y(double ry) const { return ry * h + t; }
  };

protected:
  unsigned int W;
  unsigned int H;
  unsigned int BPP;
  std::vector<T> pdata; // pixel data
public:
  typedef T PixelT;

  ImageT() : W(0), H(0), BPP(0) {}

  inline int bufoffset(const int x, const int y) const {
    return (y * W + x) * (BPP / (sizeof(T) * 8));
  }

  inline unsigned int rgb(const int x, const int y) const {
    unsigned const int off = bufoffset(x, y);
    assert(off < pdata.size());
    return pdata[off] | (pdata[off + 1] << 8) | (pdata[off + 2] << 16);
  }

  inline bool isValue(const int x, const int y, const T r, const T g,
                      const T b) const {
    const int bo = bufoffset(x, y);
    return ((pdata[bo] == r) && (pdata[bo + 1] == g) && (pdata[bo + 2] == b));
  }

  template <typename VTYPE> inline VTYPE rgbT(const int x, const int y) const {
    unsigned const int off = bufoffset(x, y);
    assert(off < pdata.size());
    return VTYPE(pdata[off], pdata[off + 1], pdata[off + 2]);
  }
  template <typename VTYPE> inline void setRGB(const int x, const int y, VTYPE rgb)
  {
      unsigned const int off = bufoffset(x, y);
      pdata[off] = rgb[0]; pdata[off+1] = rgb[1]; pdata[off+2] = rgb[2];
  }

  inline bool isValid() const { return W != 0 && H != 0 && BPP != 0; }
  inline int width() const { return W; }
  inline int height() const { return H; }
  inline int bpp() const { return BPP; }
  inline int channels() const { return BPP / (sizeof(T) * 8); }
  inline const T *data() const { return &pdata[0]; }
  inline const std::vector<T> &getData() const { return pdata; }
  inline int buffersize() const { return bufoffset(0, H); }
  inline AABB2D whole() const { return AABB2D(0, 0, W, H); }

  inline std::vector<T> &unsafeData() { return pdata; }

  inline void initialize(int width, int height, int bpp) {
    W = width;
    H = height;
    BPP = bpp;
    pdata.resize(buffersize());
  }

  inline void resize(int width, int height, int channels = 3) {
    W = width;
    H = height;
    BPP = channels * (sizeof(T) * 8);
    pdata.resize(buffersize());
  }

  // write access
  inline T &operator()(const int offset) { return pdata[offset]; }
  inline T &operator()(const int x, const int y, const int ch = 0) {
    return pdata[bufoffset(x, y) + ch];
  }

  // read access
  inline const T &operator()(const int x, const int y, const int ch = 0) const {
    return pdata[bufoffset(x, y) + ch];
  }

  // bilinear interpolation using a single channel
  template <typename VTYPE>
  inline const VTYPE bilinear(const double x, const double y) const {
      const int ix = (int)floor(x), iy = (int)floor(y);
    const int l = ix < 0 ? 0 : ix;
    const int r = l >= ((int)W - 1) ? (int)W - 1 : l + 1;
    const int t = iy < 0 ? 0 : iy;
    const int b = t >= ((int)H - 1) ? (int)H - 1 : t + 1;
    if (!((l >= 0 && l <= (int)W && t >= 0 && t <= (int)H)
        && (r >= 0 && r <= (int)W && b >= 0 && b <= (int)H)))
    {
        std::cout << "l:" << l << " t:" << t << " r:" << r << " b:" << b << std::endl;
    }
    VTYPE q11 = rgbT<VTYPE>(l, t), q21 = rgbT<VTYPE>(r, t),
          q12 = rgbT<VTYPE>(l, b), q22 = rgbT<VTYPE>(r, b);
    return q11 * (r - x) * (b - y) + q21 * (x - l) * (b - y) +
           q12 * (r - x) * (y - t) + q22 * (x - l) * (y - t);
    
  }
  // bilinear interpolation using a specific channel
  template <typename VTYPE>
  inline const VTYPE bilinear2(const double x, const double y,
                               const int ch = 0) const {
    const int l = floor(x) < 0 ? 0 : (int)floor(x);
    const int r = l >= ((int)W - 1) ? (int)W - 1 : l + 1;
    const int t = floor(y) < 0 ? 0 : (int)floor(y);
    const int b = t >= ((int)H - 1) ? (int)H - 1 : t + 1;
    assert(l >= 0 && l <= (int)W && t >= 0 && t <= (int)H);
    assert(r >= 0 && r <= (int)W && b >= 0 && b <= (int)(int)H);
    VTYPE q11 = (*this)(l, t, ch), q21 = (*this)(r, t, ch),
          q12 = (*this)(l, b, ch), q22 = (*this)(r, b, ch);
    return q11 * (r - x) * (b - y) + q21 * (x - l) * (b - y) +
           q12 * (r - x) * (y - t) + q22 * (x - l) * (y - t);
  }
  // clear image
  void clear(T value = 0) { memset(&pdata[0], value, buffersize()); }

  template <typename CALCTYPE> inline CALCTYPE mean(const int ch = 0) const {
    CALCTYPE result = 0;
    const int w = width(), h = height();
    auto MeanCB = [this, &result, ch](int x, int y) {
      result += (*this)(x, y, ch);
    };
    applyPixelPosCBS(MeanCB, whole());
    result /= (CALCTYPE)(w * h);
    return result;
  }

  template <typename CALCTYPE>
  inline CALCTYPE std(const CALCTYPE m = mean<CALCTYPE>(),
                      const int ch = 0) const {
    CALCTYPE result = 0;
    auto StdCB = [this, &result, m, ch](int x, int y) {
      CALCTYPE v = (CALCTYPE) (*this)(x, y, ch) - m;
      result += v * v;
    };
    applyPixelPosCBS(StdCB, whole());
    result /= (CALCTYPE)(W * H);
    return sqrt(result);
  }

  inline PixelT min(const int ch = 0) const {
    PixelT m = std::numeric_limits<PixelT>::max();
    auto minCB = [&m](int x, int y, PixelT v) {
      if (v < m)
        m = v;
    };
    applyPixelCB(minCB, whole());
    return m;
  }
  inline PixelT max(const int ch = 0) const {
    PixelT m = std::numeric_limits<PixelT>::min();
    auto maxCB = [&m](int x, int y, PixelT v) {
      if (v > m)
        m = v;
    };
    applyPixelCB(maxCB, whole());
    return m;
  }

  // transforms (changes) each pixel
  template <class CallBack>
  inline void transformPixelCB(const CallBack &cb, const AABB2D &wnd) {
    const int ch = channels();

    for (int y = wnd.t; y < wnd.b(); ++y) {
      for (int x = wnd.l; x < wnd.r(); ++x) {
        for (int c = 0; c < ch; ++c)
          (*this)(x, y, c) = cb((*this)(x, y, c));
      }
    }
  }

  // pixel callback (parallel)
  template <class CallBack>
  inline void applyPixelCB(const CallBack &cb, const AABB2D &wnd) const {
    const int ch = channels();

#pragma omp parallel for
    for (int y = wnd.t; y < wnd.b(); ++y) {
      for (int x = wnd.l; x < wnd.r(); ++x) {
        for (int c = 0; c < ch; ++c)
          cb(x, y, (*this)(x, y, c));
      }
    }
  }
  // pixel callback (parallel)
  template <class CallBack>
  inline void applyPixelPosCB(const CallBack &cb, const AABB2D wnd) const {
#pragma omp parallel for
    for (int y = wnd.t; y < wnd.b(); ++y) {
      for (int x = wnd.l; x < wnd.r(); ++x) {
        cb(x, y);
      }
    }
  }
  // pixel callback (not parallel)
  template <class CallBack>
  inline void applyPixelPosCBS(const CallBack &cb, const AABB2D wnd) const {
    for (int y = wnd.t; y < wnd.b(); ++y) {
      for (int x = wnd.l; x < wnd.r(); ++x) {
        cb(x, y);
      }
    }
  }
};

typedef ImageT<unsigned char> Image;
typedef ImageT<float> ImageF;
typedef ImageT<double> ImageD;

template <class SRCTYPE, class DSTTYPE>
DSTTYPE convertImage(const SRCTYPE &img, double factor = 1.0) {
  DSTTYPE result;
  const int w = img.width(), h = img.height(), c = img.channels();
  result.resize(w, h, img.channels());
  auto CB = [](int x, int y, int c, typename SRCTYPE::PixelT v) {
    result(x, y, c) = (DSTTYPE::PixelT)(img(x, y, c) * factor);
  };
  img.applyPixelCB(CB, img.whole());

  return result;
}

//============================================= color conversion
template <class SRCTYPE, class DSTTYPE>
DSTTYPE convertToGrayScale(const SRCTYPE &img) {
  DSTTYPE result;
  const int w = img.width(), h = img.height();
  result.resize(w, h, 1);

  auto CB = [&img, &result](int x, int y) {
    typename DSTTYPE::PixelT R = img(x, y, 0);
    typename DSTTYPE::PixelT G = img(x, y, 1);
    typename DSTTYPE::PixelT B = img(x, y, 2);
    // convert using luminosity
    result(x, y) = 0.21 * R + 0.72 * G + 0.07 * B;
  };
  result.applyPixelPosCB(CB, result.whole());
  return result;
}

//============================================= resize
template <class IMGTYPE>
void resizeBlit(const IMGTYPE &src, const typename IMGTYPE::AABB2D &spos,
                IMGTYPE &dst, const typename IMGTYPE::AABB2D &dpos) {
  const int ch = src.channels();
  auto CB = [&src, &dst, &spos, &dpos, ch](int x, int y) {
    for (int c = 0; c < ch; ++c) {
      double rx = dpos.x2rel(x), ry = dpos.y2rel(y);
      double sx = spos.rel2x(rx), sy = spos.rel2y(ry);
      if (x >= 0 && x < dst.width() && y >= 0 && y < dst.height())
        dst(x, y, c) = (IMGTYPE::PixelT) src.bilinear2<double>(sx, sy, c);
    }
  };
  dst.applyPixelPosCB(CB, dpos);
}

//============================================= loading / writing using libJPEG

template <class IMGTYPE> bool loadJPEG(const char *fname, IMGTYPE &img) {
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  FILE *infile = fopen(fname, "rb");

  if (infile == NULL || ferror(infile)) {
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
  for (unsigned int i = 0; i < cinfo.image_height; ++i) {
    rowptr[i] = (&img(0, i)); //     &m_data[i * cinfo.image_width * m_channels]
  }

  // read scanlines
  while (cinfo.output_scanline < cinfo.output_height) {
    jpeg_read_scanlines(&cinfo, &rowptr[cinfo.output_scanline], 10);
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(infile);

  return true;
}

template <class IMGTYPE>
bool saveJPEG(const char *fname, const IMGTYPE &img, const int quality = 80) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *outfile;
  JSAMPROW row_pointer[1];
  int row_stride;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  if ((outfile = fopen(fname, "wb")) == NULL) {
    fprintf(stderr, "error opening file %s for writing\n", fname);
    return false;
  }
  jpeg_stdio_dest(&cinfo, outfile);

  cinfo.image_width = img.width();
  cinfo.image_height = img.height();
  cinfo.input_components = img.channels();
  cinfo.in_color_space = img.channels() == 3 ? JCS_RGB : JCS_GRAYSCALE;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);

  row_stride = img.width() * img.channels();
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer[0] = (JSAMPROW)&img.getData()[cinfo.next_scanline * row_stride];
    (void)jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  jpeg_finish_compress(&cinfo);
  fclose(outfile);
  jpeg_destroy_compress(&cinfo);
  return true;
}

#endif
