#ifndef _IFS_H_
#define _IFS_H_

// Indexed Face Set datastructure
// by Ulrich Krispel 
// ulrich.krispel@fraunhofer.at

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <iterator>

namespace IFS
{
    template< typename VTYPE >
    struct LexicographicalComparator
    {
        inline bool operator() (const VTYPE &lhs, const VTYPE &rhs) const
        {
            for (int i = 0; i < VTYPE::RowsAtCompileTime; ++i)
            {
                if (lhs[i] < rhs[i]) return true;
                if (lhs[i] > rhs[i]) return false;
            }
            return false;
        }
    };

    struct Material
    {
        std::string matname;
        std::string texturename;

        Material(const std::string &mat = "nomat", const std::string &tex = "")
            : matname(mat), texturename(tex)
        {
        }
    };

  template <class IFSVERTEX, class IFSTEXCOORD >
  struct IFS
  {
     typedef IFSVERTEX IFSVTYPE;
     typedef unsigned int IFSINDEX;

     typedef std::vector< IFSVERTEX, Eigen::aligned_allocator< IFSVERTEX > > IFSVCONTAINER;        // vertex position
     typedef std::vector< IFSTEXCOORD, Eigen::aligned_allocator< IFSTEXCOORD > > IFSTCCONTAINER;   // texture coordinate

     typedef std::map< IFSVERTEX, IFSINDEX, LexicographicalComparator<IFSVERTEX> > INDEXMAP;

     typedef std::vector< IFSINDEX > IFSFACE;
     typedef std::vector< IFSFACE > IFSFCONTAINER;
     typedef std::map<int, Material> MATERIALMAP;

     IFSVCONTAINER  vertices;
     IFSTCCONTAINER texcoordinates;
     IFSFCONTAINER  faces;
     INDEXMAP       indexmap;
     MATERIALMAP    facematerial;
     
     bool useTextureCoordinates;

     IFS() : useTextureCoordinates(false) {}

     inline bool isValid() const { return !vertices.empty(); }

     // [] operator yields vertex by index
     inline const IFSVERTEX & operator[](IFSINDEX index) const
     {
         return vertices[index];
     }


     IFSINDEX vertex2index(const IFSVERTEX &v, const IFSTEXCOORD &texCoord = IFSTEXCOORD())
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
         os << "v " << (*V)[0] << " " << (*V)[1] << " " << (*V)[2] << std::endl;
         if (ifs.useTextureCoordinates)
         {
             os << "vt " << ifs.texcoordinates[vindex][0] << " " 
                         << ifs.texcoordinates[vindex][1] << std::endl;
         }
      }
      os << std::endl;
      int fi = 0;   // face index
      const Material *lastmat = 0;
      for (IFS_T::IFSFCONTAINER::const_iterator 
          F = ifs.faces.begin(), FE = ifs.faces.end();
          F != FE; ++F, ++fi)
      {
          IFS_T::MATERIALMAP::const_iterator itmat = ifs.facematerial.find(fi);
          if (itmat != ifs.facematerial.end())
          {
              if (lastmat != &itmat->second)
              {
                  os << "usemtl " << itmat->second.matname << std::endl;
                  lastmat = &itmat->second;
              }
          }
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
   static bool exportOBJ( const IFS_T &ifs, const std::string &filebase, 
                          const std::string &header="")
   {
       std::string filename = filebase;
       filename.append(".obj");
       std::string matfilename = filebase;
       matfilename.append(".mtl");
       std::string hheader = header;
       if (!ifs.facematerial.empty())
       {
           hheader.append("\n mtllib ");
           hheader.append(matfilename);
           hheader.append("\n");
       }
       std::ofstream os(filename);
       bool success = exportOBJ(ifs, os, hheader);
       if (success) {
           // export material file
           if (!ifs.facematerial.empty())
           {
               std::ofstream osmat(matfilename);

               for (const auto &mat : ifs.facematerial)
               {
                   osmat << "newmtl " << mat.second.matname << std::endl;
                   osmat << " Ka 1.000 1.000 1.000" << std::endl;
                   osmat << " Kd 1.000 1.000 1.000" << std::endl;
                   osmat << " Ks 0.000 0.000 0.000" << std::endl;
                   osmat << " d 1.0" << std::endl;
                   osmat << " illum 2" << std::endl;
                   osmat << " map_Ka " << mat.second.texturename << std::endl;
                   osmat << " map_Kd " << mat.second.texturename << std::endl;
                   osmat << std::endl;
               }
           }
       }
       return success;
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

}
#endif
