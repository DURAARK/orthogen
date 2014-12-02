/*
 * Simple Meanshift implementation
 * 
 */

#include <set>
#include <vector>

#include "types.h"

namespace MEANSHIFT
{

template < class VEC, int D > 
class Meanshift
{
private:
    std::vector < VEC > points;
    std::vector < VEC > window;
    
    const double shift_threshold; // window is considered stable if 
                                   // shift is lesser than this threshold

public:

    Meanshift(const double sh_th=0.1) : shift_threshold(sh_th)
    {
    }

    // perform very simple and slow meanshift:
    // rectangular window, one window per point
    void calculate(const VEC3 &pmin, const VEC3 &pmax, const double windowwidth)
    {
        auto const inslab = [windowwidth](const double val, const double center) -> bool
        {
            return (val < (center - windowwidth)) || (val >(center + windowwidth));
        };

        std::vector<bool> converged;
        converged.resize(points.size(), false);

        window = points;

        bool allconverged = false;

        while (!allconverged)
        {
            allconverged = true;
            int i = 0;
            for (auto &c : window)
            {
                // only consider windows that have not converged yet
                if (!converged[i])
                {
                    std::vector<VEC3> inpoints;
                    // filter points inside window
                    for (auto const &p : points)
                    {
                      bool isInside=true;
                      for (int d=0;d<D;++d) 
                      {
                        if (!inslab(p[d],c[d]) 
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
                    VEC3 mean; for (int i=0;i<D;++i) mean[i]=0;
                    
                    for (auto const &p : points) mean += p;
                    mean /= (double)points.size();

                    VEC3 ms = mean - c;
                    double length2=0;
                    for(int i=0;i<D;++i) length2+=ms[i]*ms[i];
                    
                    if (sqrt(length2) < shift_threshold)
                    {
                        converged[i] = true;
                    }
                    else
                    {
                        allconverged = false;
                        c += mean - c;
                    }
                }
                ++i; // window index
            }
        }
    }

};

}

