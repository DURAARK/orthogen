
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

#include <cstdlib>

#include <boost/program_options.hpp>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"   // FileReadStream
#include "rapidjson/encodedstream.h"    // AutoUTFInputStream

#include "types.h"
#include "projection.h"
#include "meanshift.h"
#include "extraction.h"


namespace po = boost::program_options;

// configuration variables
double WINDOW_SIZE_NORMAL = 0.3;
double WINDOW_SIZE_DISTANCE = 0.1;

typedef struct json_token JSONToken;
typedef struct json_token *JSONTokens;

using namespace OrthoGen;

int main(int ac, char *av[]) {

  // main application state
  std::vector<SphericalPanoramaImageProjection> scans;
  std::vector<Quad3Dd> walls;

  double resolution = 1.0;  // default: 1 mm/pixel
  double scalefactor = 1.0; // m
  bool exportOBJ = false;
  bool exportSphere = false;
  bool exportQuadGeometry = false;
  std::cout << "OrthoGen orthographic image generator for DuraArk" << std::endl;
  std::cout << "developed by Fraunhofer Austria Research GmbH" << std::endl;
  std::string output = "ortho";
  std::string panopath = ".\\";
  std::string aligncmd = "panoalign.exe";

  try {
    po::options_description desc("commandline options");
    desc.add_options()
        ("help", "show this help message")
        ("walljson", po::value<std::string>(), "input wall json [.json]")
        ("e57metadata", po::value<std::string>(), "e57 metadata json [.json]")
        ("panopath", po::value<std::string>(), "path to pano images")
        ("align", po::value<std::string>(), "align executable")
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

    if (vm.count("align"))
    {
        aligncmd = vm["align"].as<std::string>();
    }

    // parse jsons
    // ================================================= E57 Metadata
    // - parse scans (pose, name, panorama)

    {
        std::string e57mfn = vm["e57metadata"].as<std::string>();
        FILE* fp = fopen(e57mfn.c_str(), "rb"); // non-Windows use "r"
        char readBuffer[256];
        rapidjson::FileReadStream bis(fp, readBuffer, sizeof(readBuffer));
        rapidjson::AutoUTFInputStream<unsigned, rapidjson::FileReadStream> eis(bis);  // wraps bis into eis
        rapidjson::Document e57;         // Document is GenericDocument<UTF8<> > 
        if (e57.ParseStream<0, rapidjson::AutoUTF<unsigned> >(eis).HasParseError())
        {
            std::cout << "[ERROR] parsing input JSON file " 
                << e57mfn << std::endl;
            return -1;
        }
        fclose(fp);

        const rapidjson::Value& e57m = e57["e57m"];
        const rapidjson::Value& arr = e57["e57m"]["e57scan"];

        if (!e57m.IsObject() || !arr.IsArray()) {
            std::cout << "[ERROR] parsing input JSON file "
                << e57mfn << std::endl;
            return -1;
        }

      {
          for (rapidjson::SizeType i = 0, ie = arr.Size(); i < ie; ++i)
          {
              const rapidjson::Value& item = arr[i];
              SphericalPanoramaImageProjection new_scan;


              new_scan.basename = item["name"].GetString();
              std::cout << "found scan " << new_scan.basename << std::endl;
              // translation
              {
                  double x = item["pose"]["translation"]["x"].GetDouble();
                  double y = item["pose"]["translation"]["y"].GetDouble();
                  double z = item["pose"]["translation"]["z"].GetDouble();
                  new_scan.setPosition(Vec3d(x, y, z));
              }
              // orientation
            {
                double w = item["pose"]["rotation"]["w"].GetDouble();
                double x = item["pose"]["rotation"]["x"].GetDouble();
                double y = item["pose"]["rotation"]["y"].GetDouble();
                double z = item["pose"]["rotation"]["z"].GetDouble();
                Quaterniond quaternion(w, x, y, z);
                new_scan.applyRotation(quaternion);
            }
            std::cout << "pose: ";
            new_scan.printPose();

            // bounds
            {
                new_scan.elevationRange[0] = item["sphericalbounds"]["elevation_minimum"].GetDouble();
                new_scan.elevationRange[1] = item["sphericalbounds"]["elevation_maximum"].GetDouble();
                new_scan.azimuthRange[0] = item["sphericalbounds"]["azimuth_minimum"].GetDouble();
                new_scan.azimuthRange[1] = item["sphericalbounds"]["azimuth_maximum"].GetDouble();
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
                    std::ostringstream cmd;
                    cmd.precision(std::numeric_limits<double>::max_digits10);
                    cmd << aligncmd << " ";
                    cmd << "--imsrc " << panopath << new_scan.basename << "_Faro.jpg ";
                    cmd << "--imdst " << panopath << new_scan.basename << "_Manual.jpg ";
                    cmd << "--outname " << panopath << new_scan.basename << "_aligned.jpg ";
                    cmd << "--selrange " << new_scan.elevationRange[0] << " " << new_scan.elevationRange[1] << " ";
                    std::system(cmd.str().c_str());
                }
            }

            scans.push_back(new_scan);
            std::cout << "-------------------------------------" << std::endl;
          }
      }
    }

    // ---------------------------------------------- WALL JSON
    {
        std::string wallfn = vm["walljson"].as<std::string>();
        FILE* fp = fopen(wallfn.c_str(), "rb"); // non-Windows use "r"
        char readBuffer[256];
        rapidjson::FileReadStream bis(fp, readBuffer, sizeof(readBuffer));
        rapidjson::AutoUTFInputStream<unsigned, rapidjson::FileReadStream> eis(bis);  // wraps bis into eis
        rapidjson::Document walljson;         // Document is GenericDocument<UTF8<> > 
        if (walljson.ParseStream<0, rapidjson::AutoUTF<unsigned> >(eis).HasParseError())
        {
            std::cout << "[ERROR] parsing input JSON file " << wallfn << std::endl;
        }
        fclose(fp);


        const rapidjson::Value& wallarr = walljson["Walls"];
        if (wallarr.IsArray())
        {
            for (rapidjson::SizeType i = 0, ie = wallarr.Size(); i < ie; ++i)
            {
                const rapidjson::Value& wall = wallarr[i];
                std::string label = wall["label"].GetString();
                if (label.compare("WALL") != 0) {
                    std::cout << "[error] parsing wall " << wall["attributes"]["id"].GetString() << std::endl;
                }
                else {
                    const rapidjson::Value& att = wall["attributes"];

                    Vec3d O(att["origin"][0].GetDouble(), 
                            att["origin"][1].GetDouble(), 
                            att["origin"][2].GetDouble());
                    Vec3d W(att["x"][0].GetDouble(),
                            att["x"][1].GetDouble(),
                            att["x"][2].GetDouble());
                    W *= att["width"].GetDouble();
                    Vec3d H(att["y"][0].GetDouble(),
                            att["y"][1].GetDouble(),
                            att["y"][2].GetDouble());
                    H *= att["height"].GetDouble();
                    Quad3Dd wallgeometry(O,O+H,O+W+H,O+W);
                    wallgeometry.id = wall["attributes"]["id"].GetString();
                    walls.push_back(wallgeometry);
                }
            }
        }
        else {
            std::cout << "[ERROR] parsing input JSON file " << wallfn << std::endl;
        }

    }

    std::cout << "# # # # # # # # # # # # #" << std::endl;
    std::cout << "parsed " << scans.size() << " scans and " << walls.size() << " walls." << std::endl;

    {
        // export walls
        myIFS quadgeometry; // ifs with orthophoto quads

        quadgeometry.useTextureCoordinates = true;
        quadgeometry.texcoordinates.push_back(Vec2d(0, 1));
        quadgeometry.texcoordinates.push_back(Vec2d(0, 0));
        quadgeometry.texcoordinates.push_back(Vec2d(1, 0));
        quadgeometry.texcoordinates.push_back(Vec2d(1, 1));

        for (auto const &wall : walls) {

            // perform orthophoto projection

            //Image orthophoto = q.performProjection(projection, resolution);

            Vec3d W = wall.W();
            Vec3d H = wall.H();
            double width = W.norm();
            double height = H.norm();
            Vec3d xdir = W; xdir.normalize();
            Vec3d ydir = H; ydir.normalize();

            Image ortho;
            ortho.initialize((int)(width / resolution) + 1, (int)(height / resolution) + 1, 24);

#pragma omp parallel for
            for (int y = 0; y < ortho.height(); ++y)
            {
                const Vec3d vecy = H * (y / (double)ortho.height());
                for (int x = 0; x < ortho.width(); ++x)
                {
                    // calculate position in world coordinates
                    const Vec3d vecx = W * (x / (double)ortho.width());
                    const Vec3d position = wall.V[0] + vecx + vecy;

                    // project color value: choose nearest scan
                    double neardist = DBL_MAX;
                    const SphericalPanoramaImageProjection *S = 0;
                    for (auto const &scan : scans)
                    {
                        double dist = (position - scan.pose.O).norm();
                        if (dist < neardist) {
                            neardist = dist;
                            S = &scan;
                        }
                    }
                    if (S) 
                    {
                        RGB color = S->getColorProjection(position);
                        // write color value
                        ortho(x, y, 0) = color[0];
                        ortho(x, y, 1) = color[1];
                        ortho(x, y, 2) = color[2];
                    }
                }
            }

            {
                std::ostringstream oss;

                oss << "ortho_" << wall.id << ".jpg";
                saveJPEG(oss.str().c_str(), ortho);

                std::cout << oss.str() << " : " << ortho.width() << "x"
                    << ortho.height() << " n: " << wall.pose.Z << std::endl;
            }

            std::ostringstream matname, texname;
            matname << "ortho" << wall.id;
            texname << "ortho_" << wall.id << ".jpg";

            if (exportQuadGeometry) {
                // create quad geometry, quads always use the same texcoordinates
                myIFS::IFSINDICES face;
                myIFS::IFSINDICES facetc;
                face.push_back(quadgeometry.vertex2index(wall.V[0]));
                facetc.push_back(0);
                face.push_back(quadgeometry.vertex2index(wall.V[1]));
                facetc.push_back(1);
                face.push_back(quadgeometry.vertex2index(wall.V[2]));
                facetc.push_back(2);
                face.push_back(quadgeometry.vertex2index(wall.V[3]));
                facetc.push_back(3);
                quadgeometry.faces.push_back(face);
                quadgeometry.facetexc.push_back(facetc);
                quadgeometry.materials.push_back(
                    IFS::Material(matname.str(), texname.str()));
                quadgeometry.facematerial.push_back(quadgeometry.materials.size() -
                    1);
                assert(quadgeometry.faces.size() == quadgeometry.facetexc.size() &&
                    quadgeometry.faces.size() ==
                    quadgeometry.facematerial.size());
            }

        }

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