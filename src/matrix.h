#ifndef _MATRIX_HPP_
#define _MATRIX_HPP_


// DxD matrix with elements of type T 
// in column major order (opengl-ready)
template <class T, int D>
class Matrix
{
      T e[D*D];    // elements
public:
    inline void identity()
    {
       for(int i=0,ie=D*D;i<ie;++i)
         e[i] = (i%D) ? 0.0 : 1.0;
    }

    // default constructor: initialize with identity
    inline Matrix()        // e defaults to zero
    {
       for(int i=0,ie=D*D; i<ie; i+=D+1) e[i]=1.0;
    }

    /// conversion to pointer
    inline const T* ptr() const       { return e; }		// method access
    inline operator const T* () const { return e; }     // cast

    //! array access operator
    inline T& operator()(int row, int col) {
       assert((row >= 0) && (row < D));   // bounds check
       assert((col >= 0) && (col < D));   // bounds check
       return e[col*D+row];
    }
    inline const T& operator()(int row, int col) const {
       assert((row >= 0) && (row < D));   // bounds check
       assert((col >= 0) && (col < D));   // bounds check
       return e[col*D+row];
    } 

    inline T& operator()(int index) {
       assert((index >= 0) && (index < (D*D))); // bounds check
       return e[index];
    }

    /// matrix - vector multiplication

    // TODO: homogenous coordinates check!
    template <class V>
    inline V operator*(const V &v)
    {
        V m;  // result
        if (V::TYPE_D == D)
        {
            for (int r = 0; r < D; ++r)  // row
            {
                m[r] = 0;
                for (int d = 0; d < D; ++d) { m[r] += e[d*D + r] * v[d]; }
            }
        }
        if (V::TYPE_D == D - 1)
        {
            for (int r = 0; r < D-1; ++r)  // row
            {
                m[r] = 0;
                for (int d = 0; d < D-1; ++d) { m[r] += e[d*D + r] * v[d]; }
                m[r] += e[(D-1)*D + r];   // w=1
            }
            // calculate w
            T w = 0;
            for (int d = 0; d < D-1; ++d) { w += e[d*D + D-1] * v[d]; }
            w += e[D*D - 1];
            // divide out homogenous factor
            for (int r = 0; r < D - 1; ++r)
                m[r] /= w;

        }
        return m;
    }

    // std output stream support
   friend std::ostream& operator<<(std::ostream& os, const Matrix& m) 
   {
       for (int row=0;row<D;row++)
       {
         std::cout << "|";
         for (int col=0;col<D;col++)
         {
           os << m(row,col) << " ";
         }
         std::cout << "|" << std::endl;
       }  
       return os;
   }  
};

#endif