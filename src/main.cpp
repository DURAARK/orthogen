
//
// ortho view generation tool
// ulrich.krispel@fraunhofer.at
//

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <chrono>

#include <boost/program_options.hpp>

#include "types.h"
#include "projection.h"
#include "meanshift.h"
#include "extraction.h"

#include "../lib/frozen/frozen.h"

namespace po = boost::program_options;

// configuration variables
double WINDOW_SIZE_NORMAL = 0.3;
double WINDOW_SIZE_DISTANCE = 0.1;

typedef struct json_token JSONToken;
typedef struct json_token *JSONTokens;

int main(int ac, char *av[]) {

  // main application state
  std::vector<SphericalPanoramaImageProjection> scans;

  double resolution = 1.0;  // default: 1 mm/pixel
  double scalefactor = 1.0; // m
  bool exportOBJ = false;
  bool exportSphere = false;
  bool exportQuadGeometry = false;
  std::cout << "OrthoGen orthographic image generator for DuraArk" << std::endl;
  std::cout << "developed by Fraunhofer Austria Research GmbH" << std::endl;
  std::string output = "ortho";
  std::string panopath = ".\\";

  try {
    po::options_description desc("commandline options");
    desc.add_options()
        ("help", "show this help message")
        ("walljson", po::value<std::string>(), "input wall json [.json]")
        ("e57metadata", po::value<std::string>(), "e57 metadata json [.json]")
        ("panopath", po::value<std::string>(), "path to pano images")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc,
                                     po::command_line_style::unix_style ^
                                     po::command_line_style::allow_short),
              vm);
    po::notify(vm);

    if (vm.count("help")) //|| (!vm.count("walljson")) || (!vm.count("e57metadata")))
    {
      std::cout << desc << "\n";
      return 0;
    }

    if (vm.count("panopath"))
    {
        panopath = vm["panopath"].as<std::string>();
        std::string last = panopath.substr(panopath.length() - 1, 1);
        if (!(last.compare("\\") == 0 || last.compare("/") == 0))
        {
            panopath.append("/");
        }
    }


    // parse jsons
    // ================================================= E57 Metadata
    // - parse scans (pose, name, panorama)

    {
      std::ifstream file(vm["e57metadata"].as<std::string>().c_str(),
                         std::ios::in | std::ios::ate);
      std::vector<char> content;
      if (file.is_open()) {
        std::streampos size = file.tellg();
        content.resize(size);
        file.seekg(0, std::ios::beg);
        file.read(&content[0], size);
        file.close();
      } else {
        std::cout << "[ERROR] could not read input file "
                  << vm["e57metadata"].as<std::string>() << std::endl;
        return -1;
      }
      JSONTokens tok_e57;
      tok_e57 = parse_json2(&content[0], content.size());

      int i = 0;
      JSONTokens scan;

      auto getScan = [&scan, &tok_e57](const int i) -> JSONTokens {
        std::ostringstream ss;
        ss << "e57m.e57scan[" << i << "]";
        scan = find_json_token(tok_e57, ss.str().c_str());
        return scan;
      };
      auto tok2str = [&content](const JSONTokens tok) -> std::string {
          return std::string(tok->ptr, tok->ptr + tok->len);
      };
      auto tok2dbl = [&content,tok2str](const JSONTokens tok) -> double {
          if (tok) {
              std::istringstream ss(std::string(tok->ptr, tok->ptr + tok->len));
              double value;
              ss >> value;
              return value;
          }
          return 0.0;
      };
      
      while (scan = getScan(i)) 
      {
        if (JSONTokens name = find_json_token(scan, "name")) 
        {
            SphericalPanoramaImageProjection new_scan;
            new_scan.basename = tok2str(name);
            std::cout << "found scan " << new_scan.basename << std::endl;
            // translation
            {
                double x = tok2dbl(find_json_token(scan, "pose.translation.x"));
                double y = tok2dbl(find_json_token(scan, "pose.translation.y"));
                double z = tok2dbl(find_json_token(scan, "pose.translation.z"));
                new_scan.setPosition(Vec3d(x, y, z));
            }
            // orientation
            {
                double w = tok2dbl(find_json_token(scan, "pose.rotation.w"));
                double x = tok2dbl(find_json_token(scan, "pose.rotation.x"));
                double y = tok2dbl(find_json_token(scan, "pose.rotation.y"));
                double z = tok2dbl(find_json_token(scan, "pose.rotation.z"));
                Quaterniond quaternion(w,x,y,z);
                new_scan.applyRotation(quaternion);
            }
            std::cout << "pose: ";
            new_scan.printPose();

            // bounds
            {
                new_scan.elevationRange[0] = tok2dbl(find_json_token(scan, "sphericalbounds.elevation_minimum"));
                new_scan.elevationRange[1] = tok2dbl(find_json_token(scan, "sphericalbounds.elevation_maximum"));
                new_scan.azimuthRange[0] = tok2dbl(find_json_token(scan, "sphericalbounds.azimuth_minimum"));
                new_scan.azimuthRange[1] = tok2dbl(find_json_token(scan, "sphericalbounds.azimuth_maximum"));
                std::cout << "bounds: elevation [" << new_scan.elevationRange[0] << "<>" << new_scan.elevationRange[1]
                    << "] azimuth [" << new_scan.azimuthRange[0] << "<>" << new_scan.azimuthRange[1] << "]" << std::endl;
            }

            // load panoramic image (align if not found)
            {
                std::ostringstream ss;
                ss << panopath << new_scan.basename << "_aligned.jpg";
                Image img;
                if (loadJPEG(ss.str().c_str(), img)) {
                    new_scan.setPanoramicImage(img);
                }
                else {
                    std::cout << "aligned image not found, aligning..." << std::endl;
                }
            }
            scans.push_back(new_scan);
            std::cout << "-------------------------------------" << std::endl;
        }

        ++i;
      }

      free(tok_e57);
    }



    //  if (vm.count("im")) {
    //    Image img;
    //    if (loadJPEG(vm["im"].as<std::string>().c_str(), img)) {
    //      projection.setPanoramicImage(img);
    //    } else {
    //      std::cout << "Error: could not open " << vm["im"].as<std::string>()
    //                << std::endl;
    //    }
    //  }

    //  if (vm.count("ig")) {
    //    // load geometry
    //    ingeometry = IFS::loadOBJ<myIFS>(vm["ig"].as<std::string>());
    //    if (!ingeometry.isValid())
    //      std::cout << "Error: could not open " << vm["ig"].as<std::string>()
    //                << std::endl;
    //  }

    //  if (vm.count("res")) {
    //    resolution = vm["res"].as<double>();
    //  }

    //  if (vm.count("ncluster")) {
    //    WINDOW_SIZE_NORMAL = vm["ncluster"].as<double>();
    //  }

    //  if (vm.count("dcluster")) {
    //    WINDOW_SIZE_DISTANCE = vm["dcluster"].as<double>();
    //  }

    //  if (vm.count("trans")) {
    //    std::vector<double> trans = vm["trans"].as<std::vector<double>>();
    //    projection.setPosition(Vec3d(trans[0], trans[1], trans[2]));
    //  }

    //  if (vm.count("rot")) {
    //    std::vector<double> rot = vm["rot"].as<std::vector<double>>();
    //    if (rot.size() == 4) {
    //      Quaterniond quaternion(rot[0], rot[1], rot[2], rot[3]);
    //      std::cout << " applying quaternion [" << quaternion.x() << ","
    //                << quaternion.y() << "," << quaternion.z() << ","
    //                << quaternion.w() << "]" << std::endl;
    //      projection.applyRotation(quaternion);
    //    } else {
    //      std::cout << " --rot Error: quaternion is not composed of 4 values."
    //                << std::endl;
    //    }
    //  }

    //  if (vm.count("elevation")) {
    //    std::vector<double> el = vm["elevation"].as<std::vector<double>>();
    //    projection.elevationRange[0] = el[0];
    //    projection.elevationRange[1] = el[1];
    //  }

    //  if (vm.count("azimuth")) {
    //    std::vector<double> az = vm["azimuth"].as<std::vector<double>>();
    //    projection.azimuthRange[0] = az[0];
    //    projection.azimuthRange[1] = az[1];
    //  }

    //  if (vm.count("exgeom")) {
    //    exportOBJ = vm["exgeom"].as<bool>();
    //  }
    //  if (vm.count("exquad")) {
    //    exportQuadGeometry = vm["exgeom"].as<bool>();
    //  }
    //  if (vm.count("exsphere")) {
    //    exportSphere = vm["exsphere"].as<bool>();
    //  }
    //  if (vm.count("scale")) {
    //    std::string s = vm["scale"].as<std::string>();
    //    transform(s.begin(), s.end(), s.begin(), toupper);
    //    if (s.compare("MM") == 0)
    //      scalefactor = 1000.0;
    //    if (s.compare("CM") == 0)
    //      scalefactor = 100.0;
    //    if (s.compare("DM") == 0)
    //      scalefactor = 10.0;
    //    if (s.compare("M") == 0)
    //      scalefactor = 1.0;
    //    if (s.compare("KM") == 0)
    //      scalefactor = 0.001;
    //  }

    //  if (vm.count("output")) {
    //    output = vm["output"].as<std::string>();
    //  }
  } catch (std::exception &e) {
    std::cerr << "error: " << e.what() << "\n";
    return -1;
  }

  // if (projection.isValid() && ingeometry.isValid()) {
  //  const Image &img = projection.img();
  //  std::cout << "input image is " << img.width() << " x " << img.height()
  //            << " pixels." << std::endl;
  //  std::cout << "input geometry consists of " << ingeometry.vertices.size()
  //            << " vertices and " << ingeometry.faces.size() << " faces."
  //            << std::endl;
  //  std::cout << "using a resolution of " << resolution << "mm/pixel"
  //            << std::endl;

  //  // set up resolution scale from mm to actual scale / pixel
  //  resolution = resolution * scalefactor / 1000;

  //  // PROCESS OUTPUT
  //  if (exportSphere) {
  //    double radius1m = 0.5 * scalefactor;
  //    std::cout << "- exporting projected panorama OBJ.." << std::endl;
  //    myIFS sphere = projection.exportTexturedSphere(radius1m, 100);
  //    sphere.materials.push_back(IFS::Material("sphere", "sphere.jpg"));
  //    sphere.facematerial.push_back(sphere.materials.size() - 1);
  //    IFS::exportOBJ(sphere, "sphere", "# OrthoGen panoramic sphere\n");
  //    saveJPEG("sphere.jpg", projection.img());

  //    // projection.exportPointCloud(radius1m, 1000);
  //  }

  //  // EXTRACT QUADS FROM GEOMETRY
  //  std::cout << "Using a meanshift size of " << WINDOW_SIZE_NORMAL
  //            << " for normal clustering." << std::endl;
  //  std::cout << "Using a meanshift size of " << WINDOW_SIZE_DISTANCE
  //            << " for plane distance clustering." << std::endl;

  //  std::vector<Quad3Dd> quads;
  //  std::vector<Triangle> triangles;
  //  extract_quads(ingeometry, scalefactor, triangles, quads,
  //                WINDOW_SIZE_DISTANCE, WINDOW_SIZE_NORMAL);

  //  myIFS outgeometry; // ifs with input geometry with texture coords
  //  outgeometry.useTextureCoordinates = true;

  //  std::cout << "Exporting " << quads.size() << " ortho views." << std::endl;
  //  {
  //    myIFS quadgeometry; // ifs with orthophoto quads

  //    quadgeometry.useTextureCoordinates = true;
  //    quadgeometry.texcoordinates.push_back(Vec2d(0, 1));
  //    quadgeometry.texcoordinates.push_back(Vec2d(0, 0));
  //    quadgeometry.texcoordinates.push_back(Vec2d(1, 0));
  //    quadgeometry.texcoordinates.push_back(Vec2d(1, 1));
  //    int qid = 0;
  //    for (auto const &q : quads) {
  //      // create orthophoto projection
  //      Image orthophoto = q.performProjection(projection, resolution);
  //      {
  //        std::ostringstream oss;
  //        if (quads.size() > 1)
  //          oss << output << "_" << qid << ".jpg";
  //        else
  //          oss << output << ".jpg";
  //        saveJPEG(oss.str().c_str(), orthophoto);
  //        std::cout << oss.str() << " : " << orthophoto.width() << "x"
  //                  << orthophoto.height() << " n: " << q.pose.Z << std::endl;
  //      }

  //      std::ostringstream matname, texname;
  //      matname << "ortho" << qid;
  //      texname << "ortho_" << qid << ".jpg";

  //      if (exportQuadGeometry) {
  //        // create quad geometry, quads always use the same texcoordinates
  //        myIFS::IFSINDICES face;
  //        myIFS::IFSINDICES facetc;
  //        face.push_back(quadgeometry.vertex2index(q.V[0]));
  //        facetc.push_back(0);
  //        face.push_back(quadgeometry.vertex2index(q.V[1]));
  //        facetc.push_back(1);
  //        face.push_back(quadgeometry.vertex2index(q.V[2]));
  //        facetc.push_back(2);
  //        face.push_back(quadgeometry.vertex2index(q.V[3]));
  //        facetc.push_back(3);
  //        quadgeometry.faces.push_back(face);
  //        quadgeometry.facetexc.push_back(facetc);
  //        quadgeometry.materials.push_back(
  //            IFS::Material(matname.str(), texname.str()));
  //        quadgeometry.facematerial.push_back(quadgeometry.materials.size() -
  //                                            1);
  //        assert(quadgeometry.faces.size() == quadgeometry.facetexc.size() &&
  //               quadgeometry.faces.size() ==
  //               quadgeometry.facematerial.size());
  //      }

  //      // create output triangles for this quad
  //      if (exportOBJ) {
  //        outgeometry.materials.push_back(
  //            IFS::Material(matname.str(), texname.str()));
  //        for (size_t i : q.tri_id) {
  //          myIFS::IFSINDICES face;
  //          myIFS::IFSINDICES facetc;
  //          const Triangle &T = triangles[i];

  //          // add triangle vertices
  //          face.push_back(outgeometry.vertex2index(T.p));
  //          facetc.push_back(outgeometry.texcoordinates.size());
  //          outgeometry.texcoordinates.push_back(q.point2tex(T.p));
  //          face.push_back(outgeometry.vertex2index(T.q));
  //          facetc.push_back(outgeometry.texcoordinates.size());
  //          outgeometry.texcoordinates.push_back(q.point2tex(T.q));
  //          face.push_back(outgeometry.vertex2index(T.r));
  //          facetc.push_back(outgeometry.texcoordinates.size());
  //          outgeometry.texcoordinates.push_back(q.point2tex(T.r));

  //          // add triangle
  //          outgeometry.faces.push_back(face);
  //          outgeometry.facetexc.push_back(facetc);
  //          outgeometry.facematerial.push_back(outgeometry.materials.size() -
  //                                             1);
  //          assert(outgeometry.faces.size() == outgeometry.facetexc.size() &&
  //                 outgeometry.faces.size() ==
  //                 outgeometry.facematerial.size());
  //        }
  //      }

  //      ++qid;
  //    }

  //    if (exportQuadGeometry) {
  //      IFS::exportOBJ(quadgeometry, "quadgeometry",
  //                     "# OrthoGen textured quads\n");
  //    }
  //  }

  //  if (exportOBJ) {
  //    IFS::exportOBJ(outgeometry, "outgeometry", "# OrthoGen textured
  //    model\n");
  //  }

  //  return 0;
  //} else {
  //  if (!projection.isValid()) {
  //    std::cout << "Error: invalid panoramic image " << std::endl;
  //  }
  //  if (!ingeometry.isValid()) {
  //    std::cout << "Error: invalid geometry " << std::endl;
  //  }
  //}

  std::cout << "--help for options." << std::endl;
}