#ifndef _IFS_H_
#define _IFS_H_

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <iterator>

#include "vec3.h"

namespace IFS
{
  template <class IFSVERTEX>
  struct IFS
  {
     typedef IFSVERTEX IFSVTYPE;
     typedef unsigned int IFSINDEX;

     typedef std::vector< IFSVERTEX > IFSVCONTAINER;    // vertex position
     typedef std::vector< Vec2f >     IFSTCCONTAINER;   // texture coordinate

     typedef std::map< IFSVERTEX, IFSINDEX > INDEXMAP;

     typedef std::vector< IFSINDEX > IFSFACE;
     typedef std::vector< IFSFACE > IFSFCONTAINER;

     IFSVCONTAINER vertices;
     IFSTCCONTAINER texcoordinates;
     IFSFCONTAINER faces;
     INDEXMAP      indexmap;

     bool useTextureCoordinates;

     IFS() : useTextureCoordinates(false) {}

     inline bool isValid() const { return !vertices.empty(); }

     IFSINDEX vertex2index( const IFSVERTEX &v, const Vec2f &texCoord=Vec2f(0,0))
     {
        typename INDEXMAP::const_iterator itv = indexmap.find(v);
        if ( itv == indexmap.end())
        {
           vertices.push_back ( v );
           if (useTextureCoordinates) 
           {
               texcoordinates.push_back(texCoord);
               assert(vertices.size() == texcoordinates.size());
           }
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
  static bool exportOBJ( const IFS_T &ifs, std::ostream &os,
                         const std::string &header = "" )
  {
      os << header << std::endl;
      IFS_T::IFSINDEX vindex = 0;
      for (IFS_T::IFSVCONTAINER::const_iterator 
         V=ifs.vertices.begin(), VE=ifs.vertices.end();
         V != VE; ++V,++vindex)
      {
         os << "v " << V->x[0] << " " << V->x[1] << " " << V->x[2] << std::endl;
         if (ifs.useTextureCoordinates)
         {
             os << "vt " << ifs.texcoordinates[vindex][0] << " " 
                         << ifs.texcoordinates[vindex][1] << std::endl;
         }
      }
      os << std::endl;
      for (IFS_T::IFSFCONTAINER::const_iterator 
          F = ifs.faces.begin(), FE = ifs.faces.end();
          F != FE; ++F)
      {
         os << "f";
         for (IFS_T::IFSFACE::const_iterator it = F->begin(), ite = F->end();
            it != ite; ++it)
         {
            os << " " << (*it + 1);   // first element starts with 1!
            if (ifs.useTextureCoordinates)
            {
                os << "/" << (*it + 1);
            }
         }
         os << std::endl;
      }
      return true;
   }

   template< class IFS_T >
   static bool exportOBJ( const IFS_T &ifs, const std::string &filename, 
                          const std::string &header="")
   {
      std::ofstream os(filename);
      return exportOBJ(ifs, os, header);
   }


   void Tokenize(const std::string &str, std::vector<std::string> &tokens,
                 const std::string &delimiters = " ")
   {
       // Skip delimiters at beginning.
       std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
       // Find first "non-delimiter".
       std::string::size_type pos = str.find_first_of(delimiters, lastPos);

       while (std::string::npos != pos || std::string::npos != lastPos)
       {
           // Found a token, add it to the vector.
           tokens.push_back(str.substr(lastPos, pos - lastPos));
           // Skip delimiters.  Note the "not_of"
           lastPos = str.find_first_not_of(delimiters, pos);
           // Find next "non-delimiter"
           pos = str.find_first_of(delimiters, lastPos);
       }
   }

   template< class IFS_T >
   static IFS_T loadOBJ(std::istream &is)
   {
       IFS_T ifs;
       std::string line;

       while (getline(is, line))
       {
           // tokenize
           std::stringstream strstr(line);
           std::istream_iterator<std::string> it(strstr);
           std::istream_iterator<std::string> end;
           std::vector<std::string> linetoken(it, end);
           if (!linetoken.empty())
           {
               // VERTEX
               if ((linetoken[0].compare("v") == 0) || (linetoken[0].compare("V") == 0))
               {
                   // add vertex
                   IFS_T::IFSVTYPE vertex;
                   for (int i = 0; i < 3; ++i)
                   {
                       std::istringstream parser(linetoken[i + 1]);
                       parser >> vertex[i];
                   }
                   ifs.vertices.push_back(vertex);
               }
               // FACE
               if ((linetoken[0].compare("f") == 0) || (linetoken[0].compare("F") == 0))
               {
                   IFS_T::IFSFACE face;
                   for (unsigned i = 1; i < linetoken.size(); ++i)
                   {
                       // split by '/'
                       // vertex/texcoord/normal
                       std::vector< std::string > facevertex;
                       Tokenize(linetoken[i], facevertex, "/");
                       std::istringstream parser(facevertex[0]);    // index 0:vertex
                       unsigned vindex;
                       parser >> vindex;
                       face.push_back(vindex-1);    // first element starts with 1!
                   }
                   ifs.faces.push_back(face);
               }
               // everything else is ignored.
           }
       }

       return ifs;
   }

   
   template< class IFS_T >
   static IFS_T loadOBJ(const std::string &filename)
   {
       std::ifstream is(filename);

       return loadOBJ<IFS_T>(is);
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
