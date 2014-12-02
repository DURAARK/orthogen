

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <iomanip>

#include "types.h"
#include "projection.h"
#include "quad.h"

#include "pnm.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

typedef Quad3D<Vec3d> Quad3Dd;


int main(int ac, char* av[])
{
    SphericalPanoramaImageProjection projection;
    myIFS ingeometry;
    double resolution = 1.0;        // default: 1 mm/pixel
    double scalefactor = 1.0;       // m
    bool   exportOBJ = false;
    bool   exportSphere = false;
    std::cout << "OrthoGen orthographic image generator for DuraArk" << std::endl;

    try
    {
        po::options_description desc("commandline options");
        desc.add_options()
            ("help", "show this help message")
            ("im", po::value< std::string >(), "input panoramic image [.PNM]")
            ("ig", po::value< std::string >(), "input geometry [.OBJ]")
            ("res", po::value< double >(), "resolution [mm/pixel]")
            ("trans", po::value< std::vector<double> >()->multitoken(), "transformation [x,y,z]")
            ("rot", po::value< std::vector<double> >()->multitoken(), "rotation quaternion [w,x,y,z]")
            ("elmin", po::value< double >(), "elevation min")
            ("elmax", po::value< double >(), "elevation max")
            ("exgeom", po::value< bool >(), "export geometry [OBJ]")
            ("exsphere", po::value< bool >(), "export panoramic sphere [OBJ]")
            ("scale", po::value< std::string >(), "scale of input coordinates (mm/cm/m) ");

        po::variables_map vm;        
        po::store(po::parse_command_line(ac, av, desc, po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        if (vm.count("help")) 
        {
            std::cout << desc << "\n";
            return 0;
        }

        if (vm.count("im")) 
        {
            // load pnm image
            projection.setPanoramicImage(PNM::loadPNM(vm["im"].as<std::string>()));
            if (!projection.img().isValid())
                std::cout << "Error: could not open " 
                          << vm["im"].as<std::string>() << std::endl;
        }

        if (vm.count("ig"))
        {
            // load pnm image
            ingeometry = IFS::loadOBJ<myIFS>(vm["ig"].as<std::string>());
            if (!ingeometry.isValid())
                std::cout << "Error: could not open "
                          << vm["ig"].as<std::string>() << std::endl;
        }

        if (vm.count("res"))
        {
            resolution = vm["res"].as<double>();
        }

        if (vm.count("trans"))
        {
            std::vector<double> trans = vm["trans"].as<std::vector<double> >();
            projection.setPosition(Vec3d(trans[0], trans[1], trans[2]));
        }

        if (vm.count("rot"))
        {
            std::vector<double> rot = vm["rot"].as<std::vector<double> >();
            if (rot.size() == 4)
            {
                Quaterniond quaternion(rot[0], rot[1], rot[2], rot[3]);
                std::cout << " applying quaternion [" << quaternion.x() << ","
                    << quaternion.y() << "," << quaternion.z() << "," << quaternion.w()
                    << "]" << std::endl;
                projection.applyRotation(quaternion);
            }
            else
            {
                std::cout << " --rot Error: quaternion is not composed of 4 values." 
                          << std::endl;
            }
        }

        if (vm.count("elmin"))
        {
            // convert elevation to inclination
            projection.elevationRange[0] = vm["elmin"].as<double>();
        }
        if (vm.count("elmax"))
        {
            // convert elevation to inclination
            projection.elevationRange[1] = vm["elmax"].as<double>();
        }

        if (vm.count("exgeom"))
        {
            exportOBJ = vm["exgeom"].as<bool>();
        }

        if (vm.count("exsphere"))
        {
            exportSphere = vm["exsphere"].as<bool>();
        }
        if (vm.count("scale"))
        {
            std::string s = vm["scale"].as<std::string>();
            transform(s.begin(), s.end(), s.begin(), toupper);
            if (s.compare("MM") == 0) scalefactor = 1000.0;
            if (s.compare("CM") == 0) scalefactor = 100.0;
            if (s.compare("DM") == 0) scalefactor = 10.0;
            if (s.compare("M") == 0)  scalefactor = 1.0;
            if (s.compare("KM") == 0) scalefactor = 0.001;
        }
    }
    catch(std::exception& e) 
    {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    if (projection.isValid() && ingeometry.isValid())
    {
        const Image &img = projection.img();
        std::cout << "input image is " << img.width() << " x " << img.height() << " pixels." << std::endl;
        std::cout << "input geometry consists of " << ingeometry.vertices.size() << " vertices and " << ingeometry.faces.size() << " faces." << std::endl;
        std::cout << "using a resolution of " << resolution << "mm/pixel" << std::endl;

        // set up resolution scale from mm to actual scale / pixel
        resolution = resolution * scalefactor / 1000;


        // PROCESS OUTPUT

        if (exportSphere)
        {
            double radius1m = 0.5 * scalefactor;
            std::cout << "- exporting projected panorama OBJ.." << std::endl;
            myIFS sphere = projection.exportTexturedSphere(radius1m, 100);
            sphere.facematerial[0] = IFS::Material("sphere", "sphere.jpg");
            IFS::exportOBJ(sphere, "sphere", "# OrthoGen panoramic sphere\n");
            saveJPEG("sphere.jpg", projection.img());

            projection.exportPointCloud(radius1m, 1000);
        }

        //std::cout << "- exporting panorama pointcloud WRL.." << std::endl;

        //
        // TODO: 
        // - input E57 image to read in pose (position / orientation)
        // - model position and orientation of pano
        // for each face:
        //   for each pixel: perform ray intersection with pano
        //   write output image

        // PROJECT POINTS INTO PANO
        //Vec3d P0(-3497.68, -1339.43, 1725.98);
        //Vec2f tp = projection.world2texture(P0);
        //projection.pano((int)(tp[0] * img.width()), (int)(tp[1] * img.height()), 0) = 255;
        //projection.pano((int)(tp[0] * img.width()), (int)(tp[1] * img.height()), 1) = 0;
        //projection.pano((int)(tp[0] * img.width()), (int)(tp[1] * img.height()), 2) = 0;
        //PNM::writePNM(img, "projection.pnm");


        // UNIT TEST: spherical coordinates to texture coordinates
#ifdef UNIT_TEST_COORDINATES
        Vec3d spherical;
        Vec2d texcoord;
        auto TESTSPH = [&projection](const Vec3d spherical)
        {
            Vec2d texcoord = projection.spher2tex(spherical);
            Vec3d cartesian = projection.spher2cart(spherical);
            Vec3d spherback = projection.cart2spher(cartesian);
            double err = (spherical - spherback).norm();
            assert(err < 0.01);
            std::cout << "  X:" << cartesian[0] << " Y:" << cartesian[1] << " Z:" << cartesian[2];
            std::cout << "  A:" << spherical[0] << " E:" << spherical[1] << " R:" << spherical[2];
            std::cout << " --> [" << texcoord[0] << " , " << texcoord[1] << "]" << std::endl;
        };

        TESTSPH({ 0.0, -PI/2, 1.0 });
        TESTSPH({ 0.0, -PI/2+0.25, 1.0 });
        TESTSPH({ 0.0, 0.0, 1.0 });
        TESTSPH({ 0.0, PI / 2-0.25, 1.0 });
        TESTSPH({ 0.0, PI / 2, 1.0 });
#endif

        //// TEST WITH MANUALLY GENERATED QUAD
        //Quad3Dd quad(
        //    Vec3d(-3497.68, -1339.43, 1725.98),            
        //    Vec3d(-3497.68, -1339.43, -1532.57),
        //    Vec3d(1467.76, -1342.38, -1532.57),
        //    Vec3d(1467.76, -1342.38, 1725.98)
        //    );
        //Image orthophoto = quad.performProjection(projection, resolution);
        //{
        //    std::ostringstream oss;
        //    oss << "ortho_TEST.pnm";
        //    PNM::writePNM(orthophoto, oss.str());
        //    std::cout << oss.str() << " : " << orthophoto.width() << "x" << orthophoto.height() << std::endl;
        //}



        //std::vector<Quad3Dd> quads = extractQuads(ingeometry);

        //// export test IFS
        //myIFS quadIFS;
        //for (auto const &quad : quads)
        //{
        //    myIFS::IFSFACE f;
        //    for (auto const &v : quad.V)
        //    {
        //        f.push_back(quadIFS.vertex2index(v));
        //    }
        //    quadIFS.faces.push_back(f);

        //    //Image orthophoto = quad.performProjection(projection, resolution);
        //    //{
        //    //    std::ostringstream oss;
        //    //    oss << "ortho_" << quadIFS.faces.size() << ".pnm";
        //    //    PNM::writePNM(orthophoto, oss.str());
        //    //    std::cout << oss.str() << " : " << orthophoto.width() << "x" << orthophoto.height() << std::endl;
        //    //}

        //}

        //IFS::exportOBJ(quadIFS, "quads.obj");



        std::cout << "Exporting OrthoPhotos..." << std::endl;
        int faceid = 0;
        myIFS outgeometry = ingeometry;
        outgeometry.useTextureCoordinates = true;
        outgeometry.texcoordinates.push_back(Vec2d(0, 1));  // left top
        outgeometry.texcoordinates.push_back(Vec2d(0, 0));  // left bottom
        outgeometry.texcoordinates.push_back(Vec2d(1, 0));  // right bottom
        outgeometry.texcoordinates.push_back(Vec2d(1, 1));  // right up

        for (auto const &face : ingeometry.faces)
        {
            if (face.size() == 4)
            {
                // create quad in 3D
                Quad3Dd quad(ingeometry.vertices[face[0]], 
                             ingeometry.vertices[face[1]], 
                             ingeometry.vertices[face[2]],
                             ingeometry.vertices[face[3]]);
                // raster quad
                Image orthophoto = quad.performProjection(projection, resolution);
                {
                    std::ostringstream oss;
                    oss << "ortho_" << faceid << ".jpg";
                    //PNM::writePNM(orthophoto, oss.str());
                    saveJPEG(oss.str().c_str(), orthophoto);
                    std::cout << oss.str() << " : " << orthophoto.width() << "x" << orthophoto.height() << std::endl;
                }
                {
                    std::ostringstream matname, texname;
                    matname << "ortho" << faceid;
                    texname << "ortho_" << faceid << ".jpg";
                    outgeometry.facematerial[faceid] = IFS::Material(matname.str(), texname.str());
                    // push texture coordinates
                    myIFS::IFSINDICES texi;
                    //int tci = (quad.firstVertex+3) % 4; //(quad.firstVertex+3) % 4;
                    assert(quad.firstVertex < 4);
                    int tci = (4-quad.firstVertex) % 4;
                    for (int i = 0; i < 4; ++i, ++tci)
                    {
                        if (tci == 4) tci = 0;
                        texi.push_back(tci);
                    }
                    std::cout << quad;
                    outgeometry.facetexc.push_back(texi);
                }
            } 
            else
            {
                std::cout << "[ERR]: non-quad face detected (" << face.size() << " vertices)." << std::endl;
            }
            ++faceid;
        }


        if (exportOBJ)
        {
            // write texture coordinates for input geometry
            IFS::exportOBJ(outgeometry, "outgeometry", "# OrthoGen textured model\n");
        }

        return 0;
    }
    else 
    {
        if (!projection.isValid()) { std::cout << "Error: invalid panoramic image " << std::endl; }
        if (!ingeometry.isValid()) { std::cout << "Error: invalid geometry " << std::endl; }
    }


    std::cout << "--help for options." << std::endl;

}