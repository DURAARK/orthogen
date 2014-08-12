#ifndef _IFS_H_
#define _IFS_H_

#include <vector>
#include <map>
#include <iostream>

namespace IFS
{
  template <class IFSVERTEX>
  struct IFS
  {
     typedef unsigned int IFSINDEX;

     typedef std::vector< IFSVERTEX > IFSVCONTAINER;
     typedef std::map< IFSVERTEX, IFSINDEX > INDEXMAP;

     typedef std::vector< IFSINDEX > IFSFACE;
     typedef std::vector< IFSFACE > IFSFCONTAINER;

     IFSVCONTAINER vertices;
     IFSFCONTAINER faces;
     INDEXMAP      indexmap;

     IFSINDEX vertex2index( const IFSVERTEX &v )
     {
        typename INDEXMAP::const_iterator itv = indexmap.find(v);
        if ( itv == indexmap.end())
        {
           vertices.push_back ( v );
           indexmap[ v ] = vertices.size() - 1;
           return vertices.size() - 1;
        }
        return itv->second;
     }

     // append geometry
     void append( const IFS other )
     {
        const unsigned int vertexoffset = vertices.size();
        // append vertex data
        vertices.insert(vertices.end(), 
                        other.vertices.begin(), other.vertices.end());
        // append faces
        for (const IFSFACE &F : other.faces)
        {
           IFSFACE newface(F);
           for (IFSINDEX &I : newface)
           {
              I += vertexoffset;
           }
           faces.push_back(newface);
        }
     }
  };

  // ----------------------------------------------------------------
  // INPUT/OUTPUT METHODS
  
  template< class IFS_T >
  static bool exportToOBJ( const IFS_T &ifs, std::ostream &os )
  {
      for (IFS_T::IFSVCONTAINER::const_iterator 
         V=ifs.vertices.begin(), VE=ifs.vertices.end();
         V != VE; ++V)
      {
         os << "v " << V->x[0] << " " << V->x[1] << " " << V->x[2] << std::endl;
      }
      os << std::endl;
      for (IFS_T::IFSFCONTAINER::const_iterator 
         F=ifs.faces.begin(), ifs.FE=faces.end();
         F != FE; ++F)
      {
         os << "f";
         for (IFS_T::IFSFACE::const_iterator it = F->begin(), ite = F->end();
            it != ite; ++it)
         {
            os << " " << (*it+1);
         }
         os << std::endl;
      }
      return true;
   }

   template< class IFS_T >
   static bool exportOBJ( const IFS_T &ifs, const std::string &filename)
   {
      std::ofstream os(filename);
      exportOBJ(os);
   }

   

   // // export to .SVG (uses only first two dimensions
   // template< class IFS_T >
   // void exportSVG( const IFS_T &ifs, std::ostream &os, 
   //                 const Transformation2D &trans=Transformation2D() )
   // {
   //     os << "<svg xmlns=\"http://www.w3.org/2000/svg\"" << std::endl;
   //     os << "     xmlns:xlink=\"http://www.w3.org/1999/xlink\">" << std::endl;

   //     for (const IFSFACE &F : faces)
   //     {
   //        assert (F.size() == 2);
   //        const IFSVERTEX &from=vertices[F[0]];
   //        const IFSVERTEX &to=vertices[F[1]];
   //        double vx, vy;
   //        trans.transformVertex(from.x[0],from.x[1],vx,vy);
   //        os << "  <line x1=\"" << vx << "\"  y1=\"" << vy;
   //        trans.transformVertex(to.x[0],to.x[1],vx,vy);
   //        os << "\" x2=\""    << vx << "\"   y2=\"" << vy;
   //        os << "\" style=\"stroke:#000000\" />" << std::endl;
   //     }
   //     os << "</svg>" << std::endl;
   // }

   // void exportSVG( const std::string &filename, const Transformation2D &trans=Transformation2D())
   // {
   //    std::ofstream os(filename);
   //    exportSVG(os, trans);
   // }

}
#endif
