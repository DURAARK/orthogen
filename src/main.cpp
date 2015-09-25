
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
#include <unordered_map>
#include <unordered_set>

#include <cstdlib>

#include <boost/program_options.hpp>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"   // FileReadStream
#include "rapidjson/encodedstream.h"    // AutoUTFInputStream

#include "types.h"
#include "projection.h"
#include "meanshift.h"
#include "extraction.h"
#include "normenc.h"

namespace po = boost::program_options;

// configuration variables
double WINDOW_SIZE_NORMAL = 0.3;
double WINDOW_SIZE_DISTANCE = 0.1;

typedef struct json_token JSONToken;
typedef struct json_token *JSONTokens;

using namespace OrthoGen;

template <typename V>
void prvec(const V& v, const std::string msg="")
{
    std::cout << msg << "[" << v[0] << "," << v[1] << "," << v[2] << "]";
}

int main(int ac, char *av[]) {

  // main application state
  std::vector<SphericalPanoramaImageProjection> scans;
  std::vector<Quad3Dd> walls;
  std::unordered_map<std::string, std::unordered_set<int>> room;    // maps scans to room id

  double resolution = 0.001;                // default: 1 mm/pixel
  double walljson_scalefactor = 0.001;      // wall json is in mm
  bool exportOBJ = false, useFaroPano = false, exportRoomBB = false;
  bool verbose = false;
  int bb_normal_fit_precision = 5;

  Vec3d scan_translation_offset(0, 0, 0);
  //bool exportSphere = false;
  std::cout << "OrthoGen orthographic image generator for DuraArk" << std::endl;
  std::cout << "developed by Fraunhofer Austria Research GmbH" << std::endl;
  std::string output = "ortho";
  std::string panopath = ".\\";
  std::string aligncmd = "panoalign.exe";
  std::string specificscan = "";
  std::string specificwall = "";

  try {
    po::options_description desc("commandline options");
    desc.add_options()
        ("help", "show this help message")
        ("align", po::value<std::string>(), "align executable")
        ("bbfitnormalprecision", po::value<int>(), "normal encoding precision for oriented bounding box fit for rooms [5]")
        ("e57metadata", po::value<std::string>(), "e57 metadata json [.json]")
        ("exgeom", po::value< int >(), "export textured geometry as .obj [0]/1")
        ("exroombb", po::value< int >(), "export room bounding boxes as .obj [0]/1")
        ("output", po::value< std::string >(), "output filename [.jpg] will be appended")
        ("panopath", po::value<std::string>(), "path to pano images")
        ("resolution", po::value< double >(), "resolution [1mm/pixel]")
        ("scan", po::value<std::string>(), "if specified, only this scan will be considered")
        ("scanoffset", po::value<std::vector<double>>()->multitoken(), "translation offset")
        ("usefaroimage", po::value< int >(), "use pano from faro scanner")
        ("verbose", po::value< int >(), "print out verbose messages")
        ("wall", po::value<std::string>(), "use only this wall")
        ("walljson", po::value<std::string>(), "input wall json [.json]")
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
    if (vm.count("scan")) { specificscan = vm["scan"].as<std::string>(); }
    if (vm.count("wall")) { specificwall = vm["wall"].as<std::string>(); }
    if (vm.count("align")) { aligncmd = vm["align"].as<std::string>(); }
    if (vm.count("output")) { output = vm["output"].as<std::string>(); }
    if (vm.count("resolution")) { resolution = vm["output"].as<double>() / 1000.0; }
    if (vm.count("bbfitnormalprecision")) { bb_normal_fit_precision = vm["bbfitnormalprecision"].as<int>(); }

    if (vm.count("scanoffset"))
    {
        std::vector<double> so = vm["scanoffset"].as<std::vector<double>>();
        scan_translation_offset[0] = so[0];
        scan_translation_offset[1] = so[1];
        scan_translation_offset[2] = so[2];
    }
    exportOBJ = (vm.count("exgeom") > 0);
    exportRoomBB= (vm.count("exroombb") > 0);
    verbose = (vm.count("verbose") > 0);
    useFaroPano = (vm.count("usefaroimage") > 0);

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

              if ((specificscan.length() == 0) ||
                  ((specificscan.length() > 0) && specificscan.compare(item["name"].GetString()) == 0)
                  )
              {
                  SphericalPanoramaImageProjection new_scan;
                  new_scan.basename = item["name"].GetString();
                  std::cout << "found scan " << new_scan.basename << std::endl;
                  bool useFaro = useFaroPano;
                  // load panoramic image
                  {
                      auto loadFaro = [&new_scan, &panopath](){
                          std::ostringstream ss;
                          ss << panopath << new_scan.basename << "_Faro.jpg";
                          Image img;
                          if (loadJPEG(ss.str().c_str(), img)) {
                              new_scan.setPanoramicImage(img);
                          }
                          else {
                              std::cout << "[ERROR] loading Faro panoramic image." << std::endl;
                          }
                      };

                      if (useFaro) {
                          loadFaro();
                      }
                      else {
                          // load aligned manual image, align if not found
                          std::ostringstream ss;
                          ss << panopath << new_scan.basename << "_aligned.jpg";
                          Image img;
                          if (loadJPEG(ss.str().c_str(), img)) {
                              new_scan.setPanoramicImage(img);
                          }
                          else {
                              if (verbose) {
                                  std::cout << "aligned image not found, aligning..." << std::endl;
                              }
                              std::ostringstream cmd;
                              cmd.precision(std::numeric_limits<double>::max_digits10);
                              cmd << aligncmd << " ";
                              cmd << "--imsrc " << panopath << new_scan.basename << "_Faro.jpg ";
                              cmd << "--imdst " << panopath << new_scan.basename << "_Manual.jpg ";
                              cmd << "--outname " << panopath << new_scan.basename << "_aligned.jpg ";
                              cmd << "--selrange " << item["sphericalbounds"]["elevation_minimum"].GetDouble() << " " << item["sphericalbounds"]["elevation_maximum"].GetDouble() << " ";
                              std::cout << cmd.str();
                              std::system(cmd.str().c_str());
                              if (!loadJPEG(ss.str().c_str(), img))
                              {
                                  std::cout << "could not align image, using faro image." << std::endl;
                                  useFaro = true;
                                  loadFaro();
                              }
                          }
                      }
                  }

                  // translation
                  {
                      double x = item["pose"]["translation"]["x"].GetDouble();
                      double y = item["pose"]["translation"]["y"].GetDouble();
                      double z = item["pose"]["translation"]["z"].GetDouble();
                      new_scan.setPosition(Vec3d(x, y, z)+scan_translation_offset);
                  }
                  // orientation
                {
                    double w = item["pose"]["rotation"]["w"].GetDouble();
                    double x = item["pose"]["rotation"]["x"].GetDouble();
                    double y = item["pose"]["rotation"]["y"].GetDouble();
                    double z = item["pose"]["rotation"]["z"].GetDouble();
                    Quaterniond quaternion(w, x, y, z);
                    if (verbose) {
                        std::cout << " applying quaternion [" << quaternion.x() << ","
                            << quaternion.y() << "," << quaternion.z() << "," << quaternion.w()
                            << "]" << std::endl;
                    }
                    new_scan.applyRotation(quaternion);
                }
                if (verbose) {
                    std::cout << "pose: ";
                    new_scan.printPose();
                }

                // bounds
                {
                    if (useFaro) {
                        new_scan.elevationRange[0] = item["sphericalbounds"]["elevation_minimum"].GetDouble();
                        new_scan.elevationRange[1] = item["sphericalbounds"]["elevation_maximum"].GetDouble();
                        //new_scan.azimuthRange[0] = item["sphericalbounds"]["azimuth_minimum"].GetDouble();
                        //new_scan.azimuthRange[1] = item["sphericalbounds"]["azimuth_maximum"].GetDouble();
                    }
                    if (verbose) {
                        std::cout << "bounds: elevation [" << new_scan.elevationRange[0] << "<>" << new_scan.elevationRange[1]
                            << "] azimuth [" << new_scan.azimuthRange[0] << "<>" << new_scan.azimuthRange[1] << "]" << std::endl;
                    }
                }
                               
                if (new_scan.pano.isValid()) {
                    new_scan.id = (int)scans.size();
                    scans.push_back(new_scan);
                }
                else {
                    std::cout << "[WARNING] ignoring scan " + new_scan.basename 
                              << " because no valid pano was found." << std::endl;
                }
                if (verbose) {
                    std::cout << "-------------------------------------" << std::endl;
                }
              }
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
                    Vec3d X(att["x"][0].GetDouble(),
                            att["x"][1].GetDouble(),
                            att["x"][2].GetDouble());
                    Vec3d Y(att["y"][0].GetDouble(),
                            att["y"][1].GetDouble(),
                            att["y"][2].GetDouble());
                    double width = att["width"].GetDouble();
                    double height = att["height"].GetDouble();

                    Vec3d v0 = O*walljson_scalefactor;
                    Vec3d v1 = (O + Y*height)*walljson_scalefactor;
                    Vec3d v2 = (O + Y*height + X*width)*walljson_scalefactor;
                    Vec3d v3 = (O + X*width)*walljson_scalefactor;

                    Quad3Dd wallgeometry(v0,v1,v2,v3);
                    wallgeometry.id = wall["attributes"]["id"].GetString();
                    wallgeometry.roomid = wall["attributes"]["roomid"].GetString();
                    walls.push_back(wallgeometry);
                }
            }
        }
        else {
            std::cout << "[ERROR] parsing input JSON file " << wallfn << std::endl;
        }

    }

    // -------------------------------------------------- BUILD ROOM INDEX

    // assign scan to room if inside OBB of room walls
    {
        struct Room
        {
            std::unordered_set<int> walls;
            Pose pose;
            AABB3D bb;  // in local coordinates
        };
        std::unordered_map< std::string, Room > rooms;

        // build room index
        {
            int i = 0;
            for (auto &w : walls)
            {
                rooms[w.roomid].walls.insert(i);
                ++i;
            }
        }

        const Vec3d UP(0, 0, 1);
        NormalEncoding N(bb_normal_fit_precision);
        if (verbose) {
            std::cout << "obb fit for " << N.getMaximum() << "directions" << std::endl;
        }

        // fit room bb
        for (auto &r : rooms)
        {    
            // origin: arithmetic middle of wall coordinates
            r.second.pose.O = Vec3d(0, 0, 0);
            double s = 1.0/(r.second.walls.size() * 4);
            for (auto &wid : r.second.walls) {
                for (int i = 0; i < 4; ++i){
                    r.second.pose.O += walls[wid].V[i] * s;
                }
            }

            // kind-of-brute-force-oriented-bounding-box-fit :)
            double bestvol = DBL_MAX;
            for (int i = 0, ie = N.getMaximum(); i < ie; ++i)
            {
                for (double wg = 0; wg < 2 * M_PI; wg += (2.0*M_PI) / 10)
                {
                    Pose pose;
                    pose.O = r.second.pose.O;
                    // calculate pose
                    Quaterniond ROT, NROT;
                    ROT = Eigen::AngleAxisd::AngleAxis(wg, UP);
                    Vec3d n = N.i2n(i);
                    NROT = Quaterniond::FromTwoVectors(UP, n);
                    pose.applyRotation(NROT * ROT);

                    // calculate bounding box
                    AABB3D bb;
                    for (auto &wall : r.second.walls) {
                        for (auto &v : walls[wall].V) {
                            bb.insert(pose.world2pose(v));
                        }
                    }
                    
                    // calculate bb volume
                    if (bb.volume() < bestvol)
                    {
                        bestvol = bb.volume();
                        r.second.pose = pose;
                        r.second.bb = bb;
                    }
                }
            }
        }

        if (exportRoomBB)
        {
            myIFS roombb;            
            for (auto &r : rooms)
            {
                auto const &pose = r.second.pose;
                const Vec3d BB[2] = { r.second.bb.bbmin, r.second.bb.bbmax };
                const int MIN = 0, MAX = 1;

                myIFS::IFSINDICES face;
                auto addVertex = [&roombb, &face, &pose, &BB](const int MMX, const int MMY, const int MMZ)
                {
                    face.push_back(roombb.vertex2index(pose.pose2world(
                        Vec3d(BB[MMX][0], BB[MMY][1], BB[MMZ][2])
                    )));
                };
    
                // BOTTOM
                face.clear();
                addVertex(MIN, MIN, MIN);
                addVertex(MIN, MAX, MIN);
                addVertex(MAX, MAX, MIN);
                addVertex(MAX, MIN, MIN);
                roombb.faces.push_back(face);
                // TOP
                face.clear();
                addVertex(MIN, MIN, MAX);
                addVertex(MIN, MAX, MAX);
                addVertex(MAX, MAX, MAX);
                addVertex(MAX, MIN, MAX);
                roombb.faces.push_back(face);
                // LEFT
                face.clear();
                addVertex(MIN, MIN, MIN);
                addVertex(MIN, MAX, MIN);
                addVertex(MIN, MAX, MAX);
                addVertex(MIN, MIN, MAX);
                // RIGHT
                face.clear();
                addVertex(MAX, MIN, MIN);
                addVertex(MAX, MAX, MIN);
                addVertex(MAX, MAX, MAX);
                addVertex(MAX, MIN, MAX);
                roombb.faces.push_back(face);
                // FRONT
                face.clear();
                addVertex(MIN, MIN, MIN);
                addVertex(MAX, MIN, MIN);
                addVertex(MAX, MIN, MAX);
                addVertex(MIN, MIN, MAX);
                roombb.faces.push_back(face);
                // BACK
                face.clear();
                addVertex(MIN, MAX, MIN);
                addVertex(MAX, MAX, MIN);
                addVertex(MAX, MAX, MAX);
                addVertex(MIN, MAX, MAX);
                roombb.faces.push_back(face);
            }
            std::ostringstream objname;
            objname << output << "_rooms";
            IFS::exportOBJ(roombb, objname.str(), "# OrthoGen textured quads\n");
        }


        for (auto &s : scans)
        {
            for (auto &r : rooms)
            {
                if (r.second.bb.inside(r.second.pose.world2pose(s.pose.O)))
                {
                    room[r.first].insert(s.id);
                }
            }
        }
        if (verbose) {

            for (auto &r : room)
            {
                std::cout << "room " << r.first << " : ";
                for (auto &s : r.second)
                {
                    std::cout << scans[s].basename << " ";
                }
                std::cout << std::endl;
            }
        }
    }

    std::cout << "parsed " << scans.size() << " scans and " << walls.size() << " walls." << std::endl;
    // -------------------------------------------------- EXPORT ORTHO PHOTOS

    {
        // export walls
        myIFS quadgeometry; // ifs with orthophoto quads

        quadgeometry.useTextureCoordinates = true;
        quadgeometry.texcoordinates.push_back(Vec2d(0, 1));
        quadgeometry.texcoordinates.push_back(Vec2d(0, 0));
        quadgeometry.texcoordinates.push_back(Vec2d(1, 0));
        quadgeometry.texcoordinates.push_back(Vec2d(1, 1));

        for (auto const &wall : walls) {
            if (specificwall.length() == 0 ||
                specificwall.compare(wall.id)==0)
            {
                std::ostringstream outfile;
                outfile << output << "_" << wall.id;
                std::cout << "processing " << outfile.str() << std::endl;;
                outfile << ".jpg";

                // perform orthophoto projection
                Vec3d W = wall.W();
                Vec3d H = wall.H();
                double width = W.norm();
                double height = H.norm();
                Vec3d xdir = W; xdir.normalize();
                Vec3d ydir = H; ydir.normalize();

                Image ortho;
                ortho.initialize((int)(width / resolution) + 1, (int)(height / resolution) + 1, 24);
                if (verbose) {

                    prvec(wall.V[0], " V0:");
                    prvec(wall.V[1], " V1:");
                    std::cout << std::endl;
                    prvec(wall.V[2], " V2:");
                    prvec(wall.V[3], " V3:");
                    std::cout << std::endl;
                    std::cout << ortho.width() << "x" << ortho.height() << " n: " << wall.pose.Z << std::endl;
                }

#pragma omp parallel for
                for (int y = 0; y < ortho.height(); ++y)
                {
                    const Vec3d vecy = H * (y / (double)ortho.height());
                    for (int x = 0; x < ortho.width(); ++x)
                    {
                        // calculate position in world coordinates
                        const Vec3d vecx = W * (x / (double)ortho.width());
                        const Vec3d position = wall.V[0] + vecx + vecy;

                        // project color value: choose nearest scan, 
                        // consider only scans from this room
                        double neardist = DBL_MAX;
                        int scanid = -1;

                        for (auto &id : room[wall.roomid])
                        {
                            auto const &scan = scans[id];
                            double dist = (position - scan.pose.O).norm();
                            if (dist < neardist) {
                                neardist = dist;
                                scanid = id;
                            }
                        }
                        if (scanid != -1)
                        {
                            RGB color = scans[scanid].getColorProjection(position);
                            ortho.setRGB(x, y, color);
                        }
                    }
                }

                saveJPEG(outfile.str().c_str(), ortho);

                std::ostringstream matname, texname;
                matname << output << "_" << wall.id;
                texname << output << "_" << wall.id << ".jpg";

                if (exportOBJ) {
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
                    assert(quadgeometry.faces.size() == quadgeometry.facetexc.size()
                        && quadgeometry.faces.size() == quadgeometry.facematerial.size());
                }
            }

        }
        if (exportOBJ)
        {
            std::ostringstream objname;
            objname << output << "_geometry";
            IFS::exportOBJ(quadgeometry, objname.str(), "# OrthoGen textured quads\n");
        }
    }

  } catch (std::exception &e) {
    std::cerr << "error: " << e.what() << "\n";
    return -1;
  }

  std::cout << "--help for options." << std::endl;
}