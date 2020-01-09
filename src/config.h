#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include "types.h"
#include "projection.h"

namespace OrthoGen
{
    struct Config {
        double resolution = 0.001;                // default: 1 mm/pixel
        double walljson_scalefactor = 0.001;      // wall json is in mm
        bool exportOBJ = false, useFaroPano = false,
             exportRoomBB = false, orderCCW = true;
        bool exportOrtho = true, exportQuad = false, exportClusters;
        bool verbose = false;
        int bb_normal_fit_precision = 8;

        Vec3d scan_translation_offset = Vec3d(0, 0, 0);
		Vec3d geometry_offset = Vec3d(0, 0, 0);
		bool exportSphere = false;
        double sphereRadius = 1.0;

        // clustering parameters
        double WINDOW_SIZE_NORMAL = 0.3;
        double WINDOW_SIZE_ANGLE = 0.05;
        double WINDOW_SIZE_DISTANCE = 0.1;

        std::string e57metadatajson = "";
        std::string e57file = "";
        std::string outdir = "orthogen";
        std::string output = "ortho";
        std::string panopath = "";
        std::string aligncmd = "panoalign.exe";
        std::string specificscan = "";
        std::string specificwall = "";
        std::string jsonconfig = "";
        std::string geometryfile = "";
    };

    struct State {
        std::vector<SphericalPanoramaImageProjection> scans;
        // input geometry
        myIFS ifs;
        // triangulation
        std::vector<Triangle> triangles;
        // rectangular patches
        std::vector<Quad3Dd>  patches;

    };

namespace po = boost::program_options;

void config_parse_options(int ac, char *av[], Config &config)  {
    po::options_description desc("commandline options");
    desc.add_options()
        ("help", "show this help message")
        ("e57metadata", po::value<std::string>(), "e57 metadata json [.json] MANDATORY(A)")
        ("configjson", po::value<std::string>(), "json file with pose information MANDATORY(C)")
        ("geometry", po::value<std::string>(), "input geometry [.obj] (relative to config)")
        ("align", po::value<std::string>(), "align executable")
        ("wnddist", po::value< double >(), "distance clustering window size [0.1]")
        ("wndnorm", po::value< double >(), "normal clustering window size [0.3]")
        //("bbfitnormalprecision", po::value<int>(), "normal encoding precision for oriented bounding box fit for rooms [8]")
        ("exgeom", po::value< int >(), "export textured geometry as .obj [0]/1")
        ("exortho", po::value< int >(), "export textured geometry as .obj 0/[1]")
        ("exsphere", po::value< int >(), "export textured panoramic sphere as .obj [0]/1")
        ("exquad", po::value< int >(), "export textured panoramic sphere as .obj [0]/1")
        ("exroombb", po::value< int >(), "export room bounding boxes as .obj [0]/1")
        ("excluster", po::value< int >(), "export triangle clusters as .obj [0]/1")
		("output", po::value< std::string >(), "output filename [.jpg] will be appended")
        ("panopath", po::value<std::string>(), "path to pano images")
        ("resolution", po::value< double >(), "resolution [mm/pixel]")
        ("scan", po::value<std::string>(), "if specified, only this scan will be considered")
        ("scanoffset", po::value<std::vector<double>>()->multitoken(), "scan translation offset")
		("geometryoffset", po::value<std::vector<double>>()->multitoken(), "geometry translation offset")
		//("usefaroimage", po::value< int >(), "use pano from faro scanner")
        ("verbose", po::value< int >(), "print out verbose messages")
        //("wall", po::value<std::string>(), "use only this wall")
        //("ccw", po::value< int >(), "orientation of wall JSON quads [0]/1")
        ("sphereradius", po::value< double >(), "exported panoramic sphere radius [1.0]")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc,
        po::command_line_style::unix_style ^
        po::command_line_style::allow_short),
        vm);
    po::notify(vm);

    if (vm.count("help") )
    {
        std::cout << desc << "\n";
        std::exit(0);
    }

    if (vm.count("ccw")) {
        config.orderCCW = vm["ccw"].as<int>() != 0;
        std::cout << "json walls are ordered " << (config.orderCCW ? "CCW" : "CW") << std::endl;
    }


    if (vm.count("panopath"))
    {
        config.panopath = vm["panopath"].as<std::string>();
        std::string last = config.panopath.substr(config.panopath.length() - 1, 1);
        if (!(last.compare("\\") == 0 || last.compare("/") == 0))
        {
            config.panopath.append("/");
        }
    }
    if (vm.count("scan")) { config.specificscan = vm["scan"].as<std::string>(); }
    if (vm.count("wall")) { config.specificwall = vm["wall"].as<std::string>(); }
    if (vm.count("align")) { config.aligncmd = vm["align"].as<std::string>(); }
    if (vm.count("output")) { config.output = vm["output"].as<std::string>(); }
    if (vm.count("resolution")) { config.resolution = vm["resolution"].as<double>() / 1000.0; }
    //if (vm.count("bbfitnormalprecision")) { config.bb_normal_fit_precision = vm["bbfitnormalprecision"].as<int>(); }
    if (vm.count("sphereradius")) { config.sphereRadius = vm["sphereRadius"].as<double>(); }
    if (vm.count("wnddist")) { config.WINDOW_SIZE_DISTANCE = vm["wnddist"].as<double>(); }
    if (vm.count("wndnorm")) { config.WINDOW_SIZE_NORMAL = vm["wndnorm"].as<double>(); }
    if (vm.count("wndangle")) { config.WINDOW_SIZE_ANGLE = vm["wndangle"].as<double>(); }
    if (vm.count("e57metadata")) { config.e57metadatajson = vm["e57metadata"].as<std::string>(); }
    if (vm.count("configjson")) { config.jsonconfig = vm["configjson"].as<std::string>(); }
    if (vm.count("geometry")) { config.geometryfile = vm["geometry"].as<std::string>(); }
    if (vm.count("scanoffset"))
    {
        std::vector<double> so = vm["scanoffset"].as<std::vector<double>>();
        config.scan_translation_offset[0] = so[0];
        config.scan_translation_offset[1] = so[1];
        config.scan_translation_offset[2] = so[2];
		std::cout << "scan offset:" << config.scan_translation_offset[0] << ","
			<< config.scan_translation_offset[1] << ","
			<< config.scan_translation_offset[2] << std::endl;
    }
	if (vm.count("geometryoffset"))
	{
		std::vector<double> so = vm["geometryoffset"].as<std::vector<double>>();
		config.geometry_offset[0] = so[0];
		config.geometry_offset[1] = so[1];
		config.geometry_offset[2] = so[2];
		std::cout << "geometry offset:" << config.geometry_offset[0] << ","
			<< config.geometry_offset[1] << ","
			<< config.geometry_offset[2] << std::endl;
	}
    //if (vm.count("scale"))
    //{
    //    std::string s = vm["scale"].as<std::string>();
    //    transform(s.begin(), s.end(), s.begin(), toupper);
    //    if (s.compare("MM") == 0) config.resolution = 1000.0;
    //    if (s.compare("CM") == 0) config.resolution = 100.0;
    //    if (s.compare("DM") == 0) config.resolution = 10.0;
    //    if (s.compare("M") == 0)  config.resolution = 1.0;
    //    if (s.compare("KM") == 0) config.resolution = 0.001;
    //}
    auto parseBool = [&vm](const std::string &param) -> bool {
        return (vm.count(param) > 0) ? vm[param].as<int>() != 0 : false;
    };

    config.exportOBJ = parseBool("exgeom");
    config.exportRoomBB = parseBool("exroombb");
    config.exportSphere = parseBool("exsphere");
    config.exportOrtho= parseBool("exortho");
    config.exportQuad = parseBool("exquad");
    config.exportClusters = parseBool("excluster");
    config.verbose = parseBool("verbose");
    config.useFaroPano = parseBool("usefaroimage");
    std::cout << " normal cluster window: " << config.WINDOW_SIZE_NORMAL <<
        ", distance cluster window: " << config.WINDOW_SIZE_DISTANCE << std::endl;
}

void config_read_walljson(Config &cfg, State &state) {
    //std::string  = 
    FILE* fp = fopen(cfg.e57metadatajson.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[256];
    rapidjson::FileReadStream bis(fp, readBuffer, sizeof(readBuffer));
    rapidjson::AutoUTFInputStream<unsigned, rapidjson::FileReadStream> eis(bis);  // wraps bis into eis
    rapidjson::Document e57;         // Document is GenericDocument<UTF8<> > 
    if (e57.ParseStream<0, rapidjson::AutoUTF<unsigned> >(eis).HasParseError())
    {
        std::cout << "[ERROR] parsing input JSON file "
            << cfg.e57metadatajson << std::endl;
        std::exit(-1);
    }
    fclose(fp);

    const rapidjson::Value& e57m = e57["e57_metadata"];
    const rapidjson::Value& arr = e57["e57_metadata"]["scans"];

    if (!e57m.IsObject() || !arr.IsArray()) {
        std::cout << "[ERROR] parsing input JSON file "
            << cfg.e57metadatajson << std::endl;
        std::exit(-1);
    }

    {
        for (rapidjson::SizeType i = 0, ie = arr.Size(); i < ie; ++i)
        {
            const rapidjson::Value& item = arr[i];

            if ((cfg.specificscan.length() == 0) ||
                ((cfg.specificscan.length() > 0) && cfg.specificscan.compare(item["name"].GetString()) == 0)
                )
            {
                SphericalPanoramaImageProjection new_scan;
                new_scan.basename = item["name"].GetString();
                std::cout << "found scan " << new_scan.basename << std::endl;
                bool useFaro = cfg.useFaroPano;
                // load panoramic image
                {
                    auto loadFaro = [&new_scan, &cfg]() {
                        std::ostringstream ss;
                        ss << cfg.panopath << new_scan.basename << "_Faro.jpg";
                        Image img;
                        if (loadJPEG(ss.str().c_str(), img)) {
                            std::ostringstream pf;
                            pf << new_scan.basename << "_Faro.jpg";
                            new_scan.setPanoramicImage(pf.str(), img);
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
                        ss << cfg.panopath << new_scan.basename << "_aligned.jpg";
                        Image img;
                        if (loadJPEG(ss.str().c_str(), img)) {
                            std::ostringstream pf;
                            pf << new_scan.basename << "_aligned.jpg";
                            new_scan.setPanoramicImage(pf.str(), img);
                        }
                        else {
                            if (cfg.verbose) {
                                std::cout << "aligned image not found, aligning..." << std::endl;
                            }
                            std::ostringstream cmd;
                            cmd.precision(std::numeric_limits<double>::max_digits10);
                            cmd << cfg.aligncmd << " ";
                            cmd << "--imsrc " << cfg.panopath << new_scan.basename << "_Faro.jpg ";
                            cmd << "--imdst " << cfg.panopath << new_scan.basename << "_Manual.jpg ";
                            cmd << "--outname " << cfg.panopath << new_scan.basename << "_aligned.jpg ";
                            cmd << "--selrange " << item["spherical_bounds"]["elevation_minimum"].GetDouble() << " " << item["spherical_bounds"]["elevation_maximum"].GetDouble() << " ";
                            std::cout << cmd.str();
                            std::system(cmd.str().c_str());
                            if (!loadJPEG(ss.str().c_str(), img))
                            {
                                std::cout << "could not align image, using faro image." << std::endl;
                                useFaro = true;
                                loadFaro();
                            }
                            else {
                                std::ostringstream pf;
                                pf << new_scan.basename << "_aligned.jpg";
                                new_scan.setPanoramicImage(pf.str(), img);
                            }
                        }
                    }
                }

                // translation
                {
                    double x = item["pose"]["translation"]["x"].GetDouble();
                    double y = item["pose"]["translation"]["y"].GetDouble();
                    double z = item["pose"]["translation"]["z"].GetDouble();
                    new_scan.setPosition(Vec3d(x, y, z) + cfg.scan_translation_offset);
                }
                // orientation
                {
                    double w = item["pose"]["rotation"]["w"].GetDouble();
                    double x = item["pose"]["rotation"]["x"].GetDouble();
                    double y = item["pose"]["rotation"]["y"].GetDouble();
                    double z = item["pose"]["rotation"]["z"].GetDouble();
                    Quaterniond quaternion(w, x, y, z);
                    if (cfg.verbose) {
                        std::cout << " applying quaternion [" << quaternion.x() << ","
                            << quaternion.y() << "," << quaternion.z() << "," << quaternion.w()
                            << "]" << std::endl;
                    }
                    new_scan.applyRotation(quaternion);
                }
                if (cfg.verbose) {
                    std::cout << "pose: ";
                    new_scan.printPose();
                }

                // bounds
                {
                    if (useFaro) {
                        new_scan.elevationRange[0] = item["spherical_bounds"]["elevation_minimum"].GetDouble();
                        new_scan.elevationRange[1] = item["spherical_bounds"]["elevation_maximum"].GetDouble();
                        //new_scan.azimuthRange[0] = item["sphericalbounds"]["azimuth_minimum"].GetDouble();
                        //new_scan.azimuthRange[1] = item["sphericalbounds"]["azimuth_maximum"].GetDouble();
                    }
                    if (cfg.verbose) {
                        std::cout << "bounds: elevation [" << new_scan.elevationRange[0] << "<>" << new_scan.elevationRange[1]
                            << "] azimuth [" << new_scan.azimuthRange[0] << "<>" << new_scan.azimuthRange[1] << "]" << std::endl;
                    }
                }

                if (new_scan.pano.isValid()) {
                    new_scan.id = (int)state.scans.size();
                    state.scans.push_back(new_scan);
                }
                else {
                    std::cout << "[WARNING] ignoring scan " + new_scan.basename
                        << " because no valid pano was found." << std::endl;
                }
                if (cfg.verbose) {
                    std::cout << "-------------------------------------" << std::endl;
                }
            }
        }
    }
}   // config_read_walljson

void config_parse_json(Config &cfg, State &state) {
    FILE* fp = fopen(cfg.jsonconfig.c_str(), "rb"); // non-Windows use "r"
    if (!fp) {
		std::ostringstream ss;
		ss << "could not open config JSON : " << cfg.jsonconfig;
        throw std::exception(ss.str().c_str());
    }
    char readBuffer[256];
    rapidjson::FileReadStream bis(fp, readBuffer, sizeof(readBuffer));
    rapidjson::AutoUTFInputStream<unsigned, rapidjson::FileReadStream> eis(bis);  // wraps bis into eis
    rapidjson::Document json;         // Document is GenericDocument<UTF8<> > 
    if (json.ParseStream<0, rapidjson::AutoUTF<unsigned> >(eis).HasParseError())
    {
        std::cout << "[ERROR] parsing input JSON file "
            << cfg.jsonconfig << std::endl;
        std::exit(-1);
    }
    fclose(fp);
    //

    const rapidjson::Value& arr = json["scans"];

    if (!json.IsObject() || !arr.IsArray()) {
        std::cout << "[ERROR] parsing input JSON file "
            << cfg.jsonconfig << std::endl;
        throw std::exception("[ERROR] parsing input JSON file");
    }

    // pano path: use config setting, or try to load from json dir
    boost::filesystem::path jsonpath(cfg.jsonconfig);
    boost::filesystem::path panopath = cfg.panopath.length() == 0 ?
        jsonpath.parent_path() : cfg.panopath;

    for (rapidjson::SizeType i = 0, ie = arr.Size(); i < ie; ++i)
    {
        const rapidjson::Value& item = arr[i];
        SphericalPanoramaImageProjection new_scan;
        new_scan.basename = item["name"].GetString();
        // load panoramic image
        {
            boost::filesystem::path panofile(new_scan.basename + ".jpg");
            boost::filesystem::path imgfile = panopath / panofile;
            std::cout << "trying to load " << imgfile.string() << std::endl;
            Image img;
            if (loadJPEG(imgfile.string().c_str(), img)) {
                new_scan.setPanoramicImage(panofile.string(), img);
            }
            else {
                std::cout << "[ERROR] loading panoramic image." << std::endl;
            }
            // add pose
            // translation
            {
                new_scan.setPosition(Vec3d(
                    item["pos"]["x"].GetDouble(),
                    item["pos"]["y"].GetDouble(),
                    item["pos"]["z"].GetDouble()
                ) + cfg.scan_translation_offset);
				if (cfg.verbose) {
					std::cout << "scan position: " << new_scan.pose.O.x() << "," << new_scan.pose.O.y() << "," << new_scan.pose.O.z() << std::endl;
				}
            }
            // orientation
            {
                const Quaterniond quaternion(
                    item["rot"]["w"].GetDouble(),
                    item["rot"]["x"].GetDouble(),
                    item["rot"]["y"].GetDouble(),
                    item["rot"]["z"].GetDouble()
                );

                if (cfg.verbose) {
                    std::cout << " applying quaternion [" << quaternion.x() << ","
                        << quaternion.y() << "," << quaternion.z() << "," << quaternion.w()
                        << "]" << std::endl;
                }
                new_scan.applyRotation(quaternion);
            }
            // bounds
            new_scan.elevationRange[0] = item["bounds"]["elevation_minimum"].GetDouble();
            new_scan.elevationRange[1] = item["bounds"]["elevation_maximum"].GetDouble();
            if (cfg.verbose) {
                std::cout << "pose: ";
                new_scan.printPose();
            }

            // add image
            if (new_scan.pano.isValid()) {
                new_scan.id = (int)state.scans.size();
                state.scans.push_back(new_scan);
            }
            else {
                std::cout << "[WARNING] ignoring scan " + new_scan.basename
                    << " because no valid pano was found." << std::endl;
            }
 
        }
    }

    // OBJ geometry file
    boost::filesystem::path geometryfile = jsonpath.parent_path() / cfg.geometryfile;
    state.ifs = IFS::loadOBJ<myIFS>(geometryfile.string());

    // print some stats
    std::cout << "#################";
    std::cout << "loaded " << state.scans.size() << " scans." << std::endl;
    std::cout << "loaded geometry with " << state.ifs.faces.size() << " faces." << std::endl;
}

}// namespace OrthoGen

#endif
