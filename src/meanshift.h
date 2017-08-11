
// A simple utility class to find modes in a set of D-dimensional points
// using the meanshift algorithm.
// For use with eigen library
// ulrich.krispel@fraunhofer.at

#include <set>
#include <vector>

#include "types.h"

namespace MEANSHIFT
{

#define MEANSHIFT_MAX_ITERATIONS 20000

template < class VECD, int D > 
class Meanshift
{
private:
    
    const double shift_threshold;  // window is considered stable if 
                                    // shift is lesser than this threshold

public:
    struct VECDComparator {
        bool operator()(const VECD& a, const VECD& b) const {
            for (int d = 0; d<D; ++d)
            {
                if (a[d] < b[d]) { return true;  }
                if (a[d] > b[d]) { return false; }
            }
            return false;   // equal
        }
    };

    typedef std::vector< VECD, Eigen::aligned_allocator< VECD > > VEC;
    typedef std::map< VECD, std::vector< size_t >, VECDComparator, Eigen::aligned_allocator< std::pair<VECD, std::vector< size_t > > > > MAPPING;

    VEC points;
    VEC window;
    MAPPING cluster;

    Meanshift(const double sh_th=0.0) : shift_threshold(sh_th)
    {
    }

    // perform very simple and slow meanshift:
    // rectangular window, one window per point
    bool calculate(const double windowwidth)
    {
        if (points.empty()) return false;
        auto const inslab = [windowwidth](const double val, const double center) -> bool
        {
            return (std::abs(center-val) < windowwidth);
        };

        std::vector<bool> converged;
        converged.resize(points.size(), false);

        window = points;

        bool allconverged = false;
        int iterations = 0;

        while (!allconverged && ++iterations < MEANSHIFT_MAX_ITERATIONS)
        {
            allconverged = true;
            int i = 0;
            for (auto &c : window)
            {
                // only consider windows that have not converged yet
                if (!converged[i])
                {
                    VEC inpoints;
                    // filter points inside window
                    for (auto const &p : points)
                    {
                      bool isInside=true;
                      for (int d=0;d<D;++d) 
                      {
                        if (!inslab(p[d],c[d]) )
                        { 
                          isInside=false;
                          break;
                        }
                      }
                      if(isInside)
                      {
                        inpoints.push_back(p);
                      }
                    }
                    // calculate mean shift
                    VECD mean(VECD::Zero());
                    
                    for (auto const &p : inpoints) mean += p;
                    mean /= (double)inpoints.size();

                    VECD ms = mean - c; // mean shift vector
                    if (ms.norm() <= shift_threshold)
                    {
                        converged[i] = true;
                    }
                    else
                    {
                        allconverged = false;
                        c = mean;
                    }
                }
                ++i; // window index
            }
        }
        // assume clusters to have identical position
        if (allconverged) 
        {
            assert(points.size() == window.size());
            for (size_t i = 0, ie = window.size(); i < ie; ++i)
            {
                cluster[window[i]].push_back(i);
            }
        }

        return allconverged;
    }

};

}

