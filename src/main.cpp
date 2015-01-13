

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

#include "types.h"
#include "projection.h"
#include "quad.h"
#include "meanshift.h"

#include "pnm.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

typedef Quad3D<Vec3d> Quad3Dd;


struct Triangle
{
    const Vec3d p, q, r;      // vertices
    const Vec3d m;            // mean (midpoint)
    const Vec3d n;            // normal
    const double area;
    const int    faceid;      // triangle id 
    int cluster;              // quad id
#define TA (Q-P)
#define TB (R-P)
#define C1 (TA[1] * TB[2] - TA[2] * TB[1])
#define C2 (TA[2] * TB[0] - TA[0] * TB[2])
#define C3 (TA[0] * TB[1] - TA[1] * TB[0])

    Triangle(const Vec3d &P, const Vec3d &Q, const Vec3d &R, int face)
        : p(P), q(Q), r(R), m((P + Q + R) / 3.0), faceid(face),
        area(0.5*sqrt(C1*C1+C2*C2+C3*C3)),
        n( TA.cross(TB).normalized() )
    {
    }
};


struct AABB3D
{
    Vec3d bbmin;
    Vec3d bbmax;

    AABB3D() :
        bbmin(DBL_MAX, DBL_MAX, DBL_MAX), 
        bbmax(DBL_MIN, DBL_MIN, DBL_MIN)
    {
    }

    void insert(const Vec3d &p)
    {
        for (int i = 0; i < 3; ++i)
        {
            bbmin[i] = std::min(bbmin[i], p[i]);
            bbmax[i] = std::max(bbmax[i], p[i]);
        }
    }

};

Vec3d getPerpendicular(const Vec3d &v)
{
    Vec3d p;

    int maxdim = (std::abs(v[0]) > std::abs(v[1])) ?
        (std::abs(v[0]) > std::abs(v[2])) ? 0 : 2 :
        (std::abs(v[1]) > std::abs(v[2])) ? 1 : 2;

    switch (maxdim)
    {
    case 0:
        //px = -y - z; py = x; pz = x;
        p << -v[1] - v[2], v[0], v[0];
        break;
    case 1:
        //px = y; py = -x - z; pz = y;
        p << v[1], -v[0] - v[2], v[1];
        break;
    case 2:
        //px = z; py = z; pz = -x - y;
        p << v[2], v[2], -v[0] - v[1];
        break;
    }
    return p;
}



inline std::ostream & operator <<(std::ostream &os, const Vec3d &v) 
{
    os << std::fixed << "[" << v[0] << "," << v[1] << "," << v[2] << "]";
    return os;
}

inline std::ostream & operator <<(std::ostream &os, const AABB3D &obb)
{
    //os << "OBB X:" << obb.x << " Y:" << obb.y << " Z:" << obb.z << std::endl;
    os << "AABB min:" << obb.bbmin << " max:" << obb.bbmax << std::endl;
    return os;
}

void extract_quads(const myIFS &ifs, 
    std::vector<Triangle> &triangles,
    std::vector<Quad3Dd> &quads)
{
    std::cout.precision(2);

    // extract triangles from faces
    {
        for (auto &face : ifs.faces)
        {
            switch (face.size())
            {
            case 3: // Triangle
                triangles.push_back(Triangle(ifs.vertices[face[0]], 
                                             ifs.vertices[face[1]], 
                                             ifs.vertices[face[2]], triangles.size()));
                break;
            case 4: // Quad
                triangles.push_back(Triangle(ifs.vertices[face[0]],
                                             ifs.vertices[face[1]],
                                             ifs.vertices[face[2]], triangles.size()));

                triangles.push_back(Triangle(ifs.vertices[face[2]],
                                             ifs.vertices[face[3]],
                                             ifs.vertices[face[0]], triangles.size()));
                break;

            default:
                std::cout << "[Warning] face with " << face.size()
                    << " vertices ignored." << std::endl;

            }
        }
    }

    // cluster normals
    MEANSHIFT::Meanshift<Vec3d, 3> ms_normals;
    for (auto const &t : triangles)
    {
        ms_normals.points.push_back(t.n);
    }
    ms_normals.calculate(0.3);

    // combine clusters by implicit plane similarity: 
    // perform mean shift on cluster main direction angles

#define MS_NORMAL_CLUSTER_MAXSIZE 50
    typedef Eigen::Matrix<double, 1, MS_NORMAL_CLUSTER_MAXSIZE> Vec_NCMSd;
    assert(ms_normals.cluster.size() < MS_NORMAL_CLUSTER_MAXSIZE);

    const double MS_ANGLE_COS = cos(5.0 * M_PI / 180.0);
    // TODO: template method with max size of cluster vector
    MEANSHIFT::Meanshift<Vec_NCMSd, MS_NORMAL_CLUSTER_MAXSIZE> ms_angles;
    for (auto const &cluster1 : ms_normals.cluster)
    {
        Vec_NCMSd row = Vec_NCMSd::Zero();
        int i = 0;
        for (auto const &cluster2 : ms_normals.cluster)
        {
            double s = std::abs(cluster1.first.normalized().dot(cluster2.first.normalized()));
            row[i++] = ( s > MS_ANGLE_COS) ? 1.0 : 0.0;
        }
        ms_angles.points.push_back(row);
    }

    if (ms_angles.calculate(0.05))
    {
        std::cout << "found " << ms_angles.cluster.size() << " main directions:" << std::endl;
    }
    else {
        std::cout << "cluster did not converge" << std::endl;
    }

    // get all midpoints from all faces belonging to a direction 
    // and cluster by distance / orthogonal projection
    int clusterid = 0;
    for (auto const &clusterangle : ms_angles.cluster)
    {
        // gather triangles per directional cluster
        std::vector<Triangle> tricluster;
        {
            int i = 0;
            for (auto ncluster = ms_normals.cluster.begin(), 
                      nclustere = ms_normals.cluster.end();
                      ncluster != nclustere; ++ncluster, ++i)
            {
                // all "1" entries in the ms_angles entry
                // row correspond to a normal cluster from ms_normals
                if (clusterangle.first[i] == 1.0)
                {
                    // add all triangles from this ms_normals cluster
                    for (int id : ncluster->second)
                    {
                        tricluster.push_back(triangles[id]);
                    }
                }
            }
        }

        // TODO: maybe a better canonical representation here?
        Vec3d clusterdirection = tricluster[0].n;
        std::cout << "direction:" << clusterdirection;

        // cluster triangles by projected distance

        // faceid and point contains now all face midpoints and face id's of the 
        // cluster, project to normal direction
        MEANSHIFT::Meanshift<Vec1d, 1> ms_distance;
        for (auto const &t : tricluster) 
        {
            Vec1d d;
            d[0] = clusterdirection.dot(t.m);
            ms_distance.points.push_back(d);
        }
        ms_distance.calculate(100);
        std::cout << " clusters: " << ms_distance.cluster.size() << std::endl;

        for (auto const &dcluster : ms_distance.cluster)
        {
            Quad3Dd Q;
            // get centroid of cluster
            Vec3d center = Vec3d::Zero();
            for (const int i : dcluster.second) { center += tricluster[i].m; }
            center /= dcluster.second.size();

            // create a local coordinate system, Z = clusterdirection
            Vec3d X, Y, Z, T;
            
            Z = clusterdirection;
            Z.normalize();
            
            // create X perpendicular to Z, Y perpendicular to Z and X
            T = getPerpendicular(Z);
            X = Z.cross(T);
            X.normalize();
            Y = Z.cross(X);
            Y.normalize();
            Pose pose(center, X, Y, Z);

            // rotate around the upvector
            // eigen quaternion: w, x, y, z
            Quaterniond rot1deg(cos(0.5*M_PI / 180.0), Z[0] * sin(0.5*M_PI / 180.0), Z[1] * sin(0.5*M_PI / 180.0), Z[2] * sin(0.5*M_PI / 180.0));
            

            // OBB FIT: 
            // transform all triangle vertices into local coordinate system
            // and calculate AABB, rotate by 1° steps
            double currentarea = DBL_MAX;
            for (int deg = 0; deg < 90; ++deg)
            {
                AABB3D aabb;
                for (const int i : dcluster.second)
                {
                    const Triangle &T = tricluster[i];
                    // transform into pose coordinate system
                    aabb.insert(pose.world2pose(T.p));
                    aabb.insert(pose.world2pose(T.q));
                    aabb.insert(pose.world2pose(T.r));
                }
                Vec3d v0 = pose.pose2world(Vec3d(aabb.bbmin[0], aabb.bbmax[1], 0));
                Vec3d v1 = pose.pose2world(Vec3d(aabb.bbmin[0], aabb.bbmin[1], 0));
                Vec3d v2 = pose.pose2world(Vec3d(aabb.bbmax[0], aabb.bbmin[1], 0));
                Vec3d v3 = pose.pose2world(Vec3d(aabb.bbmax[0], aabb.bbmax[1], 0));
                Quad3Dd quad(v0, v1, v2, v3);
                if (quad.area() < currentarea)
                {
                    Q = quad;
                    currentarea = quad.area();
                    Q.pose = pose;
                }
                pose.applyRotation(rot1deg);
            }
            // mark triangles of this cluster
            for (auto ti : dcluster.second)
            {
                Q.tri_id.push_back(tricluster[ti].faceid);
            }

            quads.push_back(Q);
            //std::cout << quads.back() << std::endl;

            ++clusterid;
        }
    }

}


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
 
        std::vector<Quad3Dd> quads;
        std::vector<Triangle> triangles;
        extract_quads(ingeometry, triangles, quads);

        {
            myIFS triOBJ;
            for (auto const &T : triangles)
            {
                myIFS::IFSINDICES tri;
                tri.push_back(triOBJ.vertex2index(T.p));
                tri.push_back(triOBJ.vertex2index(T.q));
                tri.push_back(triOBJ.vertex2index(T.r));
                triOBJ.faces.push_back(tri);
            }
            IFS::exportOBJ(triOBJ,"triangles.obj");
        }
        myIFS outgeometry;                      // ifs with input geometry with texture coords
        outgeometry.useTextureCoordinates = true;

        std::cout << "Exporting " << quads.size() << "OrthoPhotos." << std::endl;
        {
            myIFS quadgeometry;                     // ifs with orthophoto quads

            quadgeometry.useTextureCoordinates = true;
            quadgeometry.texcoordinates.push_back(Vec2d(0, 1));
            quadgeometry.texcoordinates.push_back(Vec2d(0, 0));
            quadgeometry.texcoordinates.push_back(Vec2d(1, 0));
            quadgeometry.texcoordinates.push_back(Vec2d(1, 1));
            int qid = 0;
            int triid = 0;
            for (auto const &q : quads)
            {
                // create orthophoto projection
                Image orthophoto = q.performProjection(projection, resolution);
                {
                    std::ostringstream oss;
                    oss << "ortho_" << qid << ".jpg";
                    saveJPEG(oss.str().c_str(), orthophoto);
                    std::cout << oss.str() << " : " << orthophoto.width() << "x" << orthophoto.height() << std::endl;
                }

                std::ostringstream matname, texname;
                matname << "ortho" << qid;
                texname << "ortho_" << qid << ".jpg";

                {
                    // create quad geometry, quads always use the same texcoordinates
                    myIFS::IFSINDICES face;
                    myIFS::IFSINDICES facetc;
                    face.push_back(quadgeometry.vertex2index(q.V[0]));
                    facetc.push_back(0);
                    face.push_back(quadgeometry.vertex2index(q.V[1]));
                    facetc.push_back(1);
                    face.push_back(quadgeometry.vertex2index(q.V[2]));
                    facetc.push_back(2);
                    face.push_back(quadgeometry.vertex2index(q.V[3]));
                    facetc.push_back(3);
                    quadgeometry.faces.push_back(face);
                    quadgeometry.facetexc.push_back(facetc);
                    quadgeometry.facematerial[qid] = IFS::Material(matname.str(), texname.str());
                }

                // create output triangles for this quad
                for (int i : q.tri_id)
                {
                    myIFS::IFSINDICES face;
                    myIFS::IFSINDICES facetc;
                    const Triangle &T = triangles[i];

                    // add triangle vertices
                    face.push_back(outgeometry.vertex2index(T.p));
                    facetc.push_back(outgeometry.texcoordinates.size());
                    outgeometry.texcoordinates.push_back(q.point2tex(T.p));
                    face.push_back(outgeometry.vertex2index(T.q));
                    facetc.push_back(outgeometry.texcoordinates.size());
                    outgeometry.texcoordinates.push_back(q.point2tex(T.q));
                    face.push_back(outgeometry.vertex2index(T.r));
                    facetc.push_back(outgeometry.texcoordinates.size());
                    outgeometry.texcoordinates.push_back(q.point2tex(T.r));
                    
                    // add triangle
                    outgeometry.faces.push_back(face);
                    outgeometry.facetexc.push_back(facetc);
                    outgeometry.facematerial[triid] = IFS::Material(matname.str(), texname.str());
                    ++triid;
                }

                ++qid;
            }
            
            IFS::exportOBJ(quadgeometry, "quadgeometry", "# OrthoGen textured quads\n");
        }


        // export quads and create texture coordinates for input geometry

        //outgeometry.useTextureCoordinates = true;
        //outgeometry.texcoordinates.push_back(Vec2d(0, 1));  // left top
        //outgeometry.texcoordinates.push_back(Vec2d(0, 0));  // left bottom
        //outgeometry.texcoordinates.push_back(Vec2d(1, 0));  // right bottom
        //outgeometry.texcoordinates.push_back(Vec2d(1, 1));  // right up

        //std::map<int, int> face2quad;       // face id <-> quad id mapping

        //int qid = 0;
        //for (auto const &quad : quads)
        //{
        //    Image orthophoto = quad.performProjection(projection, resolution);
        //    {
        //        std::ostringstream oss;
        //        oss << "ortho_" << qid << ".jpg";
        //        saveJPEG(oss.str().c_str(), orthophoto);
        //        std::cout << oss.str() << " : " << orthophoto.width() << "x" << orthophoto.height() << std::endl;
        //    }

        //    std::ostringstream matname, texname;
        //    matname << "ortho" << qid;
        //    texname << "ortho_" << qid << ".jpg";

        //    // TODO: assign material and texture coordinates for all ifs faces
        //    //for (int faceid : quad)
        //    //outgeometry.facematerial[qid = IFS::Material(matname.str(), texname.str());

        //    ++qid;
        //}

        ////for (auto const &face : ingeometry.faces)
        ////{
        ////    if (face.size() == 4)
        ////    {
        ////        // create quad in 3D
        ////        Quad3Dd quad(ingeometry.vertices[face[0]], 
        ////                     ingeometry.vertices[face[1]], 
        ////                     ingeometry.vertices[face[2]],
        ////                     ingeometry.vertices[face[3]]);
        ////        // raster quad
        ////        Image orthophoto = quad.performProjection(projection, resolution);
        ////        {
        ////            std::ostringstream oss;
        ////            oss << "ortho_" << faceid << ".jpg";
        ////            //PNM::writePNM(orthophoto, oss.str());
        ////            saveJPEG(oss.str().c_str(), orthophoto);
        ////            std::cout << oss.str() << " : " << orthophoto.width() << "x" << orthophoto.height() << std::endl;
        ////        }
        ////        {
        ////            std::ostringstream matname, texname;
        ////            matname << "ortho" << faceid;
        ////            texname << "ortho_" << faceid << ".jpg";
        ////            outgeometry.facematerial[faceid] = IFS::Material(matname.str(), texname.str());
        ////            // push texture coordinates
        ////            myIFS::IFSINDICES texi;
        ////            //int tci = (quad.firstVertex+3) % 4; //(quad.firstVertex+3) % 4;
        ////            assert(quad.firstVertex < 4);
        ////            int tci = (4-quad.firstVertex) % 4;
        ////            for (int i = 0; i < 4; ++i, ++tci)
        ////            {
        ////                if (tci == 4) tci = 0;
        ////                texi.push_back(tci);
        ////            }
        ////            std::cout << quad;
        ////            outgeometry.facetexc.push_back(texi);
        ////        }
        ////    } 
        ////    else
        ////    {
        ////        std::cout << "[ERR]: non-quad face detected (" << face.size() << " vertices)." << std::endl;
        ////    }
        ////    ++faceid;
        ////}


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