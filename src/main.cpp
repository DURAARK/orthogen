

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <iomanip>

// use eigen matrices
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ifs.h"

#include "image.h"
#include "pnm.h"

#include "meanshift.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

// vector and  quaternion types
typedef Eigen::Quaternion<double> Quaterniond;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector2d Vec2d;

typedef Eigen::Matrix< unsigned char, 3, 1 > RGB;
typedef Eigen::Vector4i Vec4i;

// geometry
typedef IFS::IFS< Vec3d, Vec2d > myIFS;

#define PI 3.141592653589793238462643
#define PIf 3.141592653589793238462643f

//#define NEAREST_NEIGHBOR

// --------------------------------------------------------------------------
// e57 compatible camera pose 
struct Pose
{
    Vec3d O;
    Vec3d X;
    Vec3d Y;
    Vec3d Z;

    // scanner coordinate system is X-right, Y-front, Z-up
    Pose() : O(0, 0, 0), X(-1, 0, 0), Y(0, 1, 0), Z(0, 0, 1)
    {
    }

    void applyRotation(const Quaterniond &q)
    {
        X = q * X;
        Y = q * Y;
        Z = q * Z;
    }

    // std output stream support
    friend std::ostream& operator<<(std::ostream& os, const Pose& c)
    {
        os << "O: " << c.O.transpose() << " X:" << c.X.transpose() << " Y:" << c.Y.transpose() << " Z:" << c.Z.transpose() << std::endl;
        return os;
    }

    Mat4 world2pose() const
    {
        Mat4 M;
        M.col(0) = Vec4d(X[0], Y[0], Z[0], 0.0);
        M.col(1) = Vec4d(X[1], Y[1], Z[1], 0.0);
        M.col(2) = Vec4d(X[2], Y[2], Z[2], 0.0);
        M.col(3) = Vec4d(-X.dot(O), -Y.dot(O), -Z.dot(O), 1.0);
        return M;
    }

    Mat4 pose2world() const
    {
        Mat4 M;
        M.col(0) = Vec4d(X[0], X[1], X[2], 0.0);
        M.col(1) = Vec4d(Y[0], Y[1], Y[2], 0.0);
        M.col(2) = Vec4d(Z[0], Z[1], Z[2], 0.0);
        M.col(3) = Vec4d(O[0], O[1], O[2], 1.0);
        return M;
    }

};


// Image Projection allows to rayintersect with an 
// (panoramic) image taken from a known pose
class ImageProjection
{
public:
    virtual RGB getColorProjection(const Vec3d &pos) const = 0;
};

class SphericalPanoramaImageProjection : public ImageProjection
{
private:
    Pose pose;

public:
    Image pano;
    Vec2d azimuthRange;
    Vec2d elevationRange;

    SphericalPanoramaImageProjection() 
        : azimuthRange(0, 2 * PI), elevationRange(-PI/2,PI/2)
    {
    }

    inline void setPanoramicImage(const Image &pi) { pano = pi; }

    inline const Image & img()  { return pano; }

    inline bool isValid() const { return pano.isValid(); }

    inline bool setTransformation(const Vec3d &trans)
    {
        pose.O = trans;
        return true;
    }

    inline bool applyRotation(Quaterniond &quat)
    {
        std::cout << "CCS before rotation: " << pose;
        pose.applyRotation(quat);
        std::cout << "CCS after rotation: " << pose;
        return true;
    }

    // COORDINATE SYSTEM CONVERSIONS

    // spherical to texture coordinate system
    // spherical is in [azimuth | elevation | radius]
    Vec2d spher2tex(const Vec3d &spherical) const
    {
        // get relative value to  bounds
        double azimuth   = spherical[0] + PI;     // PI = scanner constant!
        double elevation = spherical[1];
        if (azimuth > 2.0*PI) azimuth -= 2.0*PI;
        Vec2d tc(
            ((azimuth - azimuthRange[0]) / (azimuthRange[1] - azimuthRange[0])),
            1.0-(elevation - elevationRange[0]) / (elevationRange[1] - elevationRange[0])
            );
        // clamp to (0,0)-(1,1)
        if (tc[0] < 0.0) tc[0] = 0.0;
        if (tc[0] > 1.0) tc[0] = 1.0;
        if (tc[1] < 0.0) tc[1] = 0.0;
        if (tc[1] > 1.0) tc[1] = 1.0;
        return tc;
    }

    // cartesian coordinates: Vec3 [ x, y, z ]
    // spherical coordinates: Vec3 [ azimuth , elevation , radius ]
    // azimuth: 0..2*pi, elevation: -PI/2..+PI/2, radius>0
    Vec3d spher2cart(const Vec3d &spc) const
    {
        // PI/2-theta because angle is elevation, not inclination
        Vec3d spherical = Vec3d(
            spc[2] * sin(PI/2-spc[1]) * cos(spc[0]),
            spc[2] * sin(PI/2-spc[1]) * sin(spc[0]),
            spc[2] * cos(PI/2-spc[1])
            );
         return spherical;
    }

    // cartesian to spherical coordinates
    Vec3d cart2spher(const Vec3d &cart) const
    {
        double radius = cart.norm();
        double elevation = PI/2 - acos(cart[2] / sqrt(cart[0]*cart[0] + cart[1]*cart[1] + cart[2]*cart[2]));
        double azimuth = atan2(cart[1], cart[0]);
        if (azimuth < 0) azimuth += 2.0*PI;

        assert(elevation >= -PI/2 && elevation <= PI/2);
        assert(azimuth >= 0.0 && azimuth <= 2.0*PI);

        return Vec3d(azimuth, elevation, radius);

    }


    Vec2d world2texture(const Vec3d &worldpos) const
    {
        Eigen::Vector4d campos = pose.world2pose() * Eigen::Vector4d(worldpos[0], worldpos[1], worldpos[2], 1.0);
        // normalize vector to get the intersection on the unit sphere
        Vec3d ray = Vec3d(campos[0] / campos[3], campos[1] / campos[3], campos[2] / campos[3]); ray.normalize();
        Vec3d spherical = cart2spher(ray);
        //spherical[0] += PI;
        //if (spherical[0] > 2.0*PI) spherical[0] -= 2.0*PI;
        return spher2tex(spherical);
    }

    RGB   getColorProjection(const Vec3d &pos) const
    {
        // transform point into camera coordinate system
        Vec2d texc = world2texture(pos);

        // pano lookup, nearest neighbor for now.. TODO: interpolation
        RGB pixel(0,0,0);
        if (texc[0] >= 0.0 && texc[0] < 1.0 && texc[1] >= 0.0 && texc[1] < 1.0)
        {
            // bilinear interpolation
#ifdef NEAREST_NEIGHBOR
            // nearest neighbor
            pixel[0] = pano((int)(texc[0] * pano.width()), (int)(texc[1] * pano.height()), 0);
            pixel[1] = pano((int)(texc[0] * pano.width()), (int)(texc[1] * pano.height()), 1);
            pixel[2] = pano((int)(texc[0] * pano.width()), (int)(texc[1] * pano.height()), 2);
#else
            // bilinear interpolation
            Vec3d pix = pano.bilinear<Vec3d>(texc[0] * (pano.width()-1), (texc[1]) * (pano.height()-1));
            pixel[0] = (unsigned char)pix[0];
            pixel[1] = (unsigned char)pix[1];
            pixel[2] = (unsigned char)pix[2];
#endif
        }
        return pixel;
    }
  
    // export to textured sphere
    myIFS exportTexturedSphere(const double r = 1.0, const int numpts = 100)
    {
        myIFS result;
        result.useTextureCoordinates = true;

        const double elevationStep = (PI / numpts);
        const double azimuthStep = (2.0*PI / numpts);


        //const Vec3d delta1(azimuthStep, 0, 0);
        //const Vec3d delta2(azimuthStep, elevationStep, 0);
        //const Vec3d delta3(0, elevationStep, 0);
        // CCW
        const Vec3d delta1(0, elevationStep, 0);
        const Vec3d delta2(azimuthStep, elevationStep, 0);
        const Vec3d delta3(azimuthStep, 0, 0);

        std::vector<Vec3d> point;
        std::vector<Vec3d> color;

        const Mat4 transform = pose.pose2world();

        //int offset = 0; // vertex index
        //for (int elIndex = 0; elIndex < numpts; elIndex++)
        //{
        //    const double e = (-PI/2) + (elIndex * PI) / (numpts-1);
        //    for (int azIndex = 0; azIndex < numpts; ++azIndex)
        //    {
        //        const double a = (azIndex * 2.0 * PI) / numpts;
        //        const Vec3d spherical(a, e, r);
        //        // convert to vertex position (cartesian)
        //        const Vec3d ccspos = spher2cart(spherical);
        //        const Vec4d pos = transform * Vec4d(ccspos[0], ccspos[1], ccspos[2], 1.0);
        //        const Vec3d v(pos[0] / pos[3], pos[1] / pos[3], pos[2] / pos[3]);

        //        result.vertices.push_back(v);

        //        if (elIndex < numpts - 1)
        //        {
        //            myIFS::IFSINDICES quad;         // vertex indices
        //            quad.push_back(offset + azIndex);                           // a,e
        //            quad.push_back(offset + (azIndex + 1) % numpts);            // a+1,e
        //            quad.push_back(offset + numpts + ((azIndex+ 1) % numpts));  // a+1,e+1
        //            quad.push_back(offset + numpts + azIndex);                  // a,e
        //            result.faces.push_back(quad);

        //        }
        //    }
        //    offset += numpts;
        //}

        //const Mat4 world2sphere = pose.world2pose();
        //// create texture coordinates
        //for (myIFS::IFSINDICES &face : result.faces)
        //{
        //    myIFS::IFSINDICES texc;            // texture coordinates
        //    for (myIFS::IFSINDEX i : face)
        //    {
        //        texc.push_back(result.texcoordinates.size());
        //        Vec4d V(result.vertices[i][0], result.vertices[i][1], result.vertices[i][2], 1.0);
        //        Vec4d VS = world2sphere * V;
        //        result.texcoordinates.push_back(spher2tex(cart2spher(Vec3d(VS[0]/VS[3], VS[1]/VS[3], VS[2]/VS[3]))));
        //    }
        //    result.facetexc.push_back(texc);
        //}



        for (double e = -PI / 2; e <= PI / 2; e += elevationStep)
        {
            for (double a = 0; a <= 2 * PI; a += azimuthStep)
            {
                const Vec3d spherical(a, e, r);
#ifdef _DEBUG
                const Vec3d cart = spher2cart(spherical);
                const Vec3d sph2 = cart2spher(cart);
                assert((spherical - sph2).norm() < 0.01); // validate conversion
                const Vec2d tex = spher2tex(spherical);
                //std::cout << " Spherical : azimuth " << spherical[0] << " elevation " << spherical[1] << " radius " << spherical[2] << std::endl;
                //std::cout << " Texture   : X " << tex[0] << " Y " << tex[1] << std::endl;
#endif

                std::set<myIFS::IFSINDEX> face;
                myIFS::IFSINDICES ifsface;
                myIFS::IFSINDICES ifsfacetc;

                auto insertVertex = [&face, &ifsface, &ifsfacetc](myIFS::IFSINDEX i)
                {
                    face.insert(i);
                    ifsface.push_back(i);
                    ifsfacetc.push_back(i);
                };

                auto cartesianVertex = [&transform, this](const Vec3d &spherical) -> Vec3d
                {
                    const Vec3d ccspos = spher2cart(spherical);
                    const Vec4d pos = transform * Vec4d(ccspos[0], ccspos[1], ccspos[2], 1.0);
                    return Vec3d(pos[0] / pos[3], pos[1] / pos[3], pos[2] / pos[3]);
                };

                // for OBJ export: bottom left origin -> v = 1.0 - v
                insertVertex(result.vertex2index(cartesianVertex(spherical), spher2tex(spherical)));
                insertVertex(result.vertex2index(cartesianVertex(spherical + delta1), spher2tex(spherical + delta1)));
                insertVertex(result.vertex2index(cartesianVertex(spherical + delta2), spher2tex(spherical + delta2)));
                insertVertex(result.vertex2index(cartesianVertex(spherical + delta3), spher2tex(spherical + delta3)));
                // process only non-degenerated faces
                if (face.size() == 4)
                {
                    result.faces.push_back(ifsface);
                    result.facetexc.push_back(ifsfacetc);
                }
            }
        }
        return result;
    }

    // export to wrl pointcloud
    myIFS exportPointCloud(const double r = 1.0, const double numpts = 100)
    {
        myIFS result;
        result.useTextureCoordinates = true;

        const double elevationStep = (PI / numpts);
        const double azimuthStep = (2.0*PI / numpts);

        const Vec3d delta1(azimuthStep, 0, 0);
        const Vec3d delta2(azimuthStep, elevationStep, 0);
        const Vec3d delta3(0, elevationStep, 0);
        
        std::vector<Vec3d> point;
        std::vector<Vec3d> color;

        const Mat4 modelview = pose.pose2world();

        std::cout << "coloring sphere points.." << std::endl;

        // create points on a sphere
        for (double e = -PI/2; e <= PI/2; e += elevationStep)
        {
            for (double a = 0; a <= 2 * PI; a += azimuthStep)
            {
                const Vec3d spherical(a, e, r);
                assert((cart2spher(spher2cart(spherical)) - spherical).norm() <= 0.01);
                const Vec3d ccspos = spher2cart(spherical);
                const Vec4d pos  = modelview * Vec4d(ccspos[0],ccspos[1],ccspos[2],1.0);
                const Vec3d wpos = Vec3d(pos[0] / pos[3], pos[1] / pos[3], pos[2] / pos[3]);
                RGB pcol = getColorProjection(wpos);
                {
                    point.push_back(wpos);
                    color.push_back(Vec3d(pcol[0], pcol[1], pcol[2]) / 255.0);
                }

            }
        }

        std::cout << "writing wrl file.." << std::endl;

        // write pointcloud file
        std::ofstream pclfile("sphere.wrl");
        pclfile << "#VRML V2.0 utf7" << std::endl << std::endl;
        pclfile << "Shape {" << std::endl;
        pclfile << "    geometry PointSet {" << std::endl;
        pclfile << "      coord Coordinate {" << std::endl;
        pclfile << "        point [ " << std::endl;
        {
            std::ostringstream ss;
            for (auto const &P : point)
            {
                ss << P[0] << " " << P[1] << " " << P[2] << ",";
            }
            pclfile << ss.str() << std::endl;
        }
        pclfile << "        ]" << std::endl; // /point
        pclfile << "      }" << std::endl; // /coordinate
        pclfile << "      colorPerVertex TRUE" << std::endl;
        pclfile << "      color Color {" << std::endl;
        pclfile << "        color [" << std::endl;
        {
            std::ostringstream ss;
            for (auto const &C : color)
            {
                ss << C[0] << " " << C[1] << " " << C[2] << ",";
            }
            pclfile << ss.str() << std::endl;
        }
        pclfile << "        ]" << std::endl; // /point
        pclfile << "      }" << std::endl; // /cikir
        pclfile << "    }" << std::endl; // /geometry
        pclfile << "}" << std::endl; // /shape

        return result;
    }
};

template< class T >
struct Quad3D
{
    T V[4];                     // quad coordinates, in CCW Order starting top left
    int firstVertex;

    // vertices are given in CCW order
    Quad3D(const T &v0, const T &v1, const T &v2, const T &v3)
    {
        // we assume that the Z direction corresponds to the up direction in world space
        // therefore, order the points such that the points with maximum z coordinate
        // are given first, without changing the orientation
        typedef std::pair<double, unsigned int> vdat;

        std::vector < vdat > brak;
        const struct bcomp {
            inline bool operator() (const vdat &A, const vdat &B) const
            {
                return A.first < B.first;
            }
        } brakComp;

        brak.push_back(vdat(v0[2] + v1[2], 1));
        brak.push_back(vdat(v1[2] + v2[2], 2));
        brak.push_back(vdat(v2[2] + v3[2], 3));
        brak.push_back(vdat(v3[2] + v0[2], 0));
        std::sort(brak.begin(), brak.end(), brakComp);
        firstVertex = brak[3].second;
        switch (firstVertex)
        {
            case 0: V[0] = v0; V[1] = v1; V[2] = v2; V[3] = v3; break;
            case 1: V[0] = v1; V[1] = v2; V[2] = v3; V[3] = v0; break;
            case 2: V[0] = v2; V[1] = v3; V[2] = v0; V[3] = v1; break;
            case 3: V[0] = v3; V[1] = v0; V[2] = v1; V[3] = v2; break;
            default:
            std::abort();
        }
    }

    // resolution is <world units>/pixel
    Image performProjection(const ImageProjection& projection, const double resolution) const
    {
#ifdef WRITE_POINTCLOUD_PER_FACE
        static int imagecounter = 0;
        std::ostringstream ss;
        ss << "cloud_" << imagecounter << ".xyz";
        std::ofstream pclfile(ss.str());
        ++imagecounter;
#endif

        // origin is left top
        T H  = V[1] - V[0];     // Y - Vector
        T W = V[3] - V[0];      // X - Vector

        double width = W.norm();     // vnorm(W);
        double height = H.norm();    // vnorm(H);

        // get image space X and Y directional vectors in world coordinates
        T xdir = W; xdir.normalize();
        T ydir = H; ydir.normalize();
        
        Image img;
        img.initialize((int)(width/resolution)+1, (int)(height/resolution)+1, 24);

        for (int y = 0; y < img.height(); ++y)
        {
            const T vecy = H * (y / (double)img.height());
            for (int x = 0; x < img.width(); ++x)
            {
                // calculate position in world coordinates
                const T vecx = W * (x / (double)img.width());
                const T position = V[0] + vecx + vecy;

                // project color value
                RGB color = projection.getColorProjection(position);

                // write color value
                img(x, y, 0) = color[0];
                img(x, y, 1) = color[1];
                img(x, y, 2) = color[2];

#ifdef WRITE_POINTCLOUD_PER_FACE
                // write pointcloud file
                std::ostringstream pclpoint;
                pclpoint << position[0] << " " << position[1] << " " << position[2];
                pclpoint << " " << (int)color[0] << " " << (int)color[1] << " " << (int)color[2];
                pclfile << pclpoint.str() << std::endl;
#endif

            }
        }
        
        return img;
    }

};

// std output stream support
template <class T> std::ostream& operator<<(std::ostream& os, const Quad3D<T> &q)
{
    auto PV = [&q](std::ostream& os, const int i) { os << "[" << q.V[i][0] << ", " << q.V[i][1] << ", " << q.V[i][2] << "] "; };
    //os << q.V[0] << " " << q.V[1] << " " << q.V[2] << q.V[3] << std::endl;
    os << std::setprecision(2);
    os << "  V0:";
    PV(os, 0);
    os << "  V1:";
    PV(os, 1);
    os << std::endl << "  V2:";
    PV(os, 2);
    os << "  V3:";
    PV(os, 3);
    os << std::endl;
    os << " first face vertex: " << q.firstVertex << std::endl;
    return os;
}

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
            projection.setTransformation(Vec3d(trans[0], trans[1], trans[2]));
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
            if (s.compare("MM") == 0) scalefactor = 0.001;
            if (s.compare("CM") == 0) scalefactor = 0.01;
            if (s.compare("DM") == 0) scalefactor = 0.1;
            if (s.compare("M") == 0) scalefactor = 1.0;
            if (s.compare("KM") == 0) scalefactor = 1000.0;
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
        
        // PROCESS OUTPUT

        if (exportSphere)
        {
            //std::cout << "- exporting projected panorama OBJ.." << std::endl;
            //myIFS sphere = projection.exportTexturedSphere(1.0 / scalefactor, 100);
            //sphere.facematerial[0] = IFS::Material("sphere", "sphere.jpg");
            //IFS::exportOBJ(sphere, "sphere", "# OrthoGen panoramic sphere\n");
            //saveJPEG("sphere.jpg", projection.img());

            //projection.exportPointCloud(1.0 / scalefactor, 1000);
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