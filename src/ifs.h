#ifndef _IFS_H_
#define _IFS_H_

// Indexed Face Set datastructure with texture coordinates
// ulrich.krispel@fraunhofer.at

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <iterator>
#include <fstream>

namespace IFS
{
    // lexicographical comparator for eigen vectors
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

    // textured material
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
     typedef IFSTEXCOORD IFSTCTYPE;
     typedef size_t IFSINDEX;

     typedef std::vector< IFSVERTEX, Eigen::aligned_allocator< IFSVERTEX > > IFSVCONTAINER;        // vertex position
     typedef std::vector< IFSTEXCOORD, Eigen::aligned_allocator< IFSTEXCOORD > > IFSTCCONTAINER;   // texture coordinate

     typedef std::map< IFSVERTEX, IFSINDEX, LexicographicalComparator<IFSVERTEX> > INDEXMAP;
     typedef std::map< IFSTEXCOORD, IFSINDEX, LexicographicalComparator<IFSTEXCOORD> > INDEXTCMAP;

     typedef std::vector< IFSINDEX >   IFSINDICES;
     typedef std::vector< IFSINDICES > IFSICONTAINER;

     typedef std::vector< Material >   IFSMATCONTAINER;
     
     IFSVCONTAINER  vertices;
     IFSVCONTAINER  normals;
     IFSTCCONTAINER texcoordinates;
     IFSMATCONTAINER materials;
     IFSICONTAINER  faces;                  // vertex indices per face
     IFSICONTAINER  facetexc;               // texture coordinate indices per face
     IFSICONTAINER  facenorm;               // face normal indices per face
     IFSINDICES  facematerial;              // material id per face

     INDEXMAP       indexmap;               // indexing of vertex coordinates
     INDEXTCMAP     indextcmap;             // indexing of texture coordinates

     bool useTextureCoordinates;
     bool useNormals;

     IFS() : useTextureCoordinates(false), useNormals(false) {}

     inline bool isValid() const { return !vertices.empty(); }

     // [] operator yields vertex by index
     inline const IFSVERTEX & operator[](IFSINDEX index) const
     {
         return vertices[index];
     }

     IFSINDEX vertex2index(const IFSVERTEX &v)
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

     IFSINDEX tc2index(const IFSTEXCOORD &tc)
     {
         typename INDEXTCMAP::const_iterator ittc = indextcmap.find(tc);
         if (ittc == indextcmap.end())
         {
             texcoordinates.push_back(tc);
             indextcmap[tc] = texcoordinates.size() - 1;
             return texcoordinates.size() - 1;
         }
         return ittc->second;
     }

     // append geometry
     void append( const IFS &other )
     {
        const unsigned int vertexoffset = vertices.size();
        // append vertex data
        vertices.insert(vertices.end(), 
                        other.vertices.begin(), other.vertices.end());
        // append faces
        for (const IFSINDICES &F : other.faces)
        {
           IFSINDICES newface(F);
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
	  os << std::fixed;
      {
          typename IFS_T::IFSINDEX vindex = 0;
          for (typename IFS_T::IFSVCONTAINER::const_iterator
              V = ifs.vertices.begin(), VE = ifs.vertices.end();
              V != VE; ++V, ++vindex)
          {
              os << "v " << (*V)[0] << " " << (*V)[1] << " " << (*V)[2] << std::endl;
          }
      }
      os << std::endl;
      if (ifs.useTextureCoordinates)
      {
          for (auto const &TC : ifs.texcoordinates)
          {
              os << "vt " << TC[0] << " " << TC[1] << std::endl;
          }
      }

      typename IFS_T::IFSINDEX fi = 0;   // face index
      typename IFS_T::IFSINDEX lastmatid = 0xFFFFFFFF;

      for (typename IFS_T::IFSICONTAINER::const_iterator 
          F = ifs.faces.begin(), FE = ifs.faces.end();
          F != FE; ++F, ++fi)
      {
          if (fi < ifs.facematerial.size())
          {
              if (lastmatid != ifs.facematerial[fi])
              {
                  if (ifs.facematerial[fi] < ifs.materials.size()) {
                      const Material &mat = ifs.materials[ifs.facematerial[fi]];
                      os << "usemtl " << mat.matname << std::endl;
                      lastmatid = ifs.facematerial[fi];
                  }
              }

          }
         os << "f";
         typename IFS_T::IFSINDICES::const_iterator tcit;
         if (ifs.useTextureCoordinates)
         {
             tcit = ifs.facetexc[fi].begin();
         }
         for (typename IFS_T::IFSINDICES::const_iterator it = F->begin(), ite = F->end();
            it != ite; ++it)
         {
            os << " " << (*it + 1);   // obj starts counting by 1...
            if (ifs.useTextureCoordinates)
            {
                os << "/" << (*tcit + 1);
                ++tcit;
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

	   // export matfile in the same path
       std::string matfilename = filebase;
       matfilename.append(".mtl");

	   const boost::filesystem::path matfilepath(matfilename);

       std::string hheader = header;
       if (!ifs.facematerial.empty())
       {
           hheader.append("\n mtllib ");
           hheader.append(matfilepath.filename().string());
           hheader.append("\n");
       }
       std::ofstream os(filename);
       bool success = exportOBJ(ifs, os, hheader);
       if (success) {
           // export material file
           if (!ifs.facematerial.empty())
           {
               std::ofstream osmat(matfilename);

               for (const auto &mat : ifs.materials)
               {
                   osmat << "newmtl " << mat.matname << std::endl;
                   osmat << " Ka 1.000 1.000 1.000" << std::endl;
                   osmat << " Kd 1.000 1.000 1.000" << std::endl;
                   osmat << " Ks 0.000 0.000 0.000" << std::endl;
                   osmat << " d 1.0" << std::endl;
                   osmat << " illum 2" << std::endl;
                   osmat << " map_Ka " << mat.texturename << std::endl;
                   osmat << " map_Kd " << mat.texturename << std::endl;
                   osmat << std::endl;
               }
           }
       }
       return success;
   }

   inline void Tokenize(const std::string &str, 
                         std::vector<std::string> &tokens,
                         const std::string &delimiters = " ")
   {
       std::string::size_type last = str.find_first_not_of(delimiters, 0);
       std::string::size_type i = str.find_first_of(delimiters, last);

       while (std::string::npos != last || std::string::npos != i)
       {
           tokens.push_back(str.substr(last, i - last));
           last = str.find_first_not_of(delimiters, i);
           i = str.find_first_of(delimiters, last);
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
                   typename IFS_T::IFSVTYPE vertex;
                   for (int i = 0; i < 3; ++i)
                   {
                       std::istringstream parser(linetoken[i + 1]);
                       parser >> vertex[i];
                   }
                   ifs.vertices.push_back(vertex);
               }
               // TEXTURE COORDINATE
               if ((linetoken[0].compare("vt") == 0) || (linetoken[0].compare("VT") == 0))
               {
                   // add tc
                   typename IFS_T::IFSTCTYPE tc;
                   for (int i = 0; i < 2; ++i)
                   {
                       std::istringstream parser(linetoken[i + 1]);
                       parser >> tc[i];
                   }
                   ifs.texcoordinates.push_back(tc);
               }
               // NORMAL
               if ((linetoken[0].compare("vn") == 0) || (linetoken[0].compare("VN") == 0))
               {
                   // add vertex
                   typename IFS_T::IFSVTYPE normal;
                   for (int i = 0; i < 3; ++i)
                   {
                       std::istringstream parser(linetoken[i + 1]);
                       parser >> normal[i];
                   }
                   ifs.normals.push_back(normal);
               }
               // FACE
               if ((linetoken[0].compare("f") == 0) || (linetoken[0].compare("F") == 0))
               {
                   typename IFS_T::IFSINDICES face;
                   typename IFS_T::IFSINDICES facetc;
                   typename IFS_T::IFSINDICES facenormal;
                   for (unsigned i = 1; i < linetoken.size(); ++i)
                   {
                       // split by '/'
                       // vertex/texcoord/normal
                       std::vector< std::string > facevertex;
                       Tokenize(linetoken[i], facevertex, "/");
                       {
                           std::istringstream parser(facevertex[0]);    // index 0:vertex
                           unsigned vindex;
                           parser >> vindex;
                           face.push_back(vindex - 1);    // first element starts with 1!
                       }
					   if (facevertex.size() > 1)
                       {
                           if (!facevertex[1].empty()) {
                               std::istringstream parser(facevertex[1]);    // index 1:texcoord
                               unsigned tcindex;
                               parser >> tcindex;
                               facetc.push_back(tcindex - 1);    // first element starts with 1!
                           }
                       }
					   if(facevertex.size() > 2)
                       {
                           if (!facevertex[2].empty()) {
                               std::istringstream parser(facevertex[2]);    // index 1:normal
                               unsigned nindex;
                               parser >> nindex;
                               facenormal.push_back(nindex - 1);    // first element starts with 1!
                           }
                       }
                   }
                   ifs.faces.push_back(face);
                   if (!facetc.empty()) {
                       ifs.facetexc.push_back(facetc);
                   }
                   if (!facenormal.empty()) {
                       ifs.facenorm.push_back(facenormal);
                   }
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
       if (!is.is_open()) {
           std::cout << "[ERROR]: could not open " << filename << std::endl;
       }
       return loadOBJ<IFS_T>(is);
   }

}
#endif
