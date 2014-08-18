
#include "image.h"
#include "pnm.h"
#include "ifs.h"
#include "vec3.h"
#include "camera.h"
#include "quaternion.h"

#include <vector>
#include <string>

#include <cmath>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

typedef IFS::IFS< Vec3d > myIFS;

typedef MathVector<unsigned char,3> RGB;    // RGB color value
#define PI 3.141592653589793238462643




// Image Projection allows to rayintersect with an 
// (panoramic) image taken from a known pose
class ImageProjection
{
public:
    virtual Vec3d getPosition() = 0;
    virtual RGB   getColorProjection(const Vec3d &pos) const = 0;
};


class SphericalPanoramaImageProjection : public ImageProjection
{
private:
    Camera<double> camera;
    Image pano;

public:
    Vec2d azimuthRange;
    Vec2d elevationRange;

    SphericalPanoramaImageProjection() 
        : azimuthRange(0, 2 * PI), elevationRange(-PI/2,PI/2)
    {
    }

    inline void setPanoramicImage(const Image &pi) { pano = pi; }
    inline void setCenter(const Vec3d &center)     { camera.setPosition(center); }
    inline void setPose(const Vec3d &right, const Vec3d &up, const Vec3d &front) 
    { 
        camera.setOrientation(right, up, front);
    }

    inline const Image & img()  { return pano; }

    inline bool isValid() const { return pano.isValid(); }
    inline Vec3d getPosition()  { return camera.getPosition(); }

    inline bool applyRotation(Quaterniond &quat)
    {
        camera.applyRotation(quat);
        return true;
    }


    RGB   getColorProjection(const Vec3d &pos) const
    {
        // get vector from camera center to position
        Vec3d rayWorld = pos - camera.getPosition();

        // transform to camera orientation
        Vec3d ray = camera.world2cam() * rayWorld;

        // normalize vector to get the intersection on the unit sphere
        ray.normalize();

        // pano-x = azimuth   = phi   = atan2(y,x) normalized to [0,1]
        // pano-y = elevation = theta = arccos(z/sqrt(x^2+y^2+z^2)) normalized to [-1,1], norm can be neglected 
        //                                         since (x,y,z) is already normalized
        // physical notation, theta=inclination/elevation=pano-x ,phi = azimuth = pano-y
        Vec3d spherical = cart2spher(ray);

        // get relative value to  bounds
        const double tx = (spherical[0] - azimuthRange[0]) / (azimuthRange[1] - azimuthRange[0]);
        const double ty = (spherical[1] - elevationRange[0]) / (elevationRange[1] - elevationRange[0]);

        // pano lookup, nearest neighbor for now.. TODO: interpolation
        RGB pixel(0,0,0);
        if (tx >= 0.0 && tx <= 1.0 && ty >= 0.0 && ty <= 1.0)
        {
            pixel[0] = pano(tx*pano.width(), ty*pano.height(), 0);
            pixel[1] = pano(tx*pano.width(), ty*pano.height(), 1);
            pixel[2] = pano(tx*pano.width(), ty*pano.height(), 2);
        }
        return pixel;
    }

    // COORDINATE SYSTEM CONVERSIONS

    // cartesian coordinates: Vec3 [ x, y, z ]
    // spherical coordinates: Vec3 [ azimuth , inclination , radius ]
    Vec3d spher2cart(const Vec3d &spc) const
    {    
        return Vec3d(
            spc[2] * sin(spc[1]) * cos(spc[0]),
            spc[2] * sin(spc[1]) * sin(spc[0]),
            spc[2] * cos(spc[1])
        );
    }

    // cartesian to spherical coordinates
    Vec3d cart2spher(const Vec3d &cart) const
    {
        double radius    = cart.length();
        double inclination = acos(cart[2] / radius);
        double azimuth   = atan2(cart[1], cart[0]);
        if (azimuth < 0) azimuth += 2.0*PI;

        assert(inclination >= 0.0 && inclination <= PI);
        assert(azimuth >= 0.0 && azimuth <= 2.0*PI);

        return Vec3d(azimuth, inclination, radius);

    }
    
    // export to textured sphere
    myIFS exportToIFS(const double r = 1.0, const double numpts = 10)
    {
        myIFS result;
        result.useTextureCoordinates = true;

        const double inclinationStep = (PI / numpts);
        const double azimuthStep = (2.0*PI / numpts);

        const Vec3d delta1(azimuthStep, 0, 0);
        const Vec3d delta2(azimuthStep, inclinationStep, 0);
        const Vec3d delta3(0, inclinationStep, 0);

        const Vec2f txd1(azimuthStep, 0);
        const Vec2f txd2(azimuthStep, inclinationStep);
        const Vec2f txd3(0, inclinationStep);
        const Vec2f txnorm(1.0/(2.0*PI), 1.0/PI);

        // create points on a sphere
        for (double e = 0; e < PI; e += inclinationStep)
        {
            for (double a = 0; a < 2 * PI; a += azimuthStep)
            {
                const Vec3d spherical(a, e, r);
                const Vec2f texcoord(a, e);

                std::set<myIFS::IFSINDEX> face;
                myIFS::IFSFACE ifsface;

                auto insertVertex = [&face, &ifsface](myIFS::IFSINDEX i)
                {
                    face.insert(i);
                    ifsface.push_back(i);
                };

                insertVertex(result.vertex2index(spher2cart(spherical), texcoord.elMul(txnorm)));
                insertVertex(result.vertex2index(spher2cart(spherical + delta1), (texcoord + txd1).elMul(txnorm)));
                insertVertex(result.vertex2index(spher2cart(spherical + delta2), (texcoord + txd2).elMul(txnorm)));
                insertVertex(result.vertex2index(spher2cart(spherical + delta3), (texcoord + txd3).elMul(txnorm)));
                // process only non-degenerated faces
                if (face.size() == 4)
                {
                    result.faces.push_back(ifsface);
                }
            }
        }

        return result;
    }
};


template< class T >
struct Quad3D
{
    T V[4];

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

        brak.push_back(vdat(v0[2] + v1[2], 0));
        brak.push_back(vdat(v1[2] + v2[2], 1));
        brak.push_back(vdat(v2[2] + v3[2], 2));
        brak.push_back(vdat(v3[2] + v0[2], 3));
        std::sort(brak.begin(), brak.end(), brakComp);
        switch (brak[3].second)
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
    Image performProjection(const ImageProjection& projection, double resolution) const
    {
#ifdef WRITE_POINTCLOUD_PER_FACE
        static int imagecounter = 0;
        std::ostringstream ss;
        ss << "cloud_" << imagecounter << ".xyz";
        std::ofstream pclfile(ss.str());
        ++imagecounter;
#endif

        // origin is left top
        T W  = V[1] - V[0];
        T H = V[3] - V[0];

        double width = W.length();
        double height = H.length();

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
typedef Quad3D<Vec3d> Quad3Dd;

// TODO: additional projection types?
//       - fisheye camera dewarping
//       - HomographyProjection from a planar image


int main(int ac, char* av[])
{
    SphericalPanoramaImageProjection projection;
    myIFS ingeometry;
    double resolution = 1.0;        // default: 1 mm/pixel

    std::cout << "OrthoGen orthographic image generator for DuraArk" << std::endl;

    try
    {
        po::options_description desc("commandline options");
        desc.add_options()
            ("help", "show this help message")
            ("im", po::value< std::string >(), "input panoramic image [.PNM]")
            ("ig", po::value< std::string >(), "input geometry [.OBJ]")
            ("res", po::value< double >(), "resolution [mm/pixel]")
            ("rot", po::value< std::vector<double> >()->multitoken(), "rotation quaternion [x,y,z,w]")
            ("elmin", po::value< double >(), "elevation min")
            ("elmax", po::value< double >(), "elevation max")
            ;

        po::variables_map vm;        
        po::store(po::parse_command_line(ac, av, desc), vm);
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

        if (vm.count("rot"))
        {
            std::vector<double> rot = vm["rot"].as<std::vector<double> >();
            if (rot.size() == 4)
            {
                Quaterniond quaternion(rot[0], rot[1], rot[2], rot[3]);
                std::cout << " applying quaternion [" << quaternion.a << ","
                    << quaternion.b << "," << quaternion.c << "," << quaternion.d
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
            projection.elevationRange[0] = vm["elmin"].as<double>();
        }
        if (vm.count("elmax"))
        {
            projection.elevationRange[1] = vm["elmax"].as<double>();
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
        //PNM::writePNM(img, "projection.pnm");
        std::cout << "input image is " << img.width() << " x " << img.height() << " pixels." << std::endl;
        std::cout << "input geometry consists of " << ingeometry.vertices.size() << " vertices and " << ingeometry.faces.size() << " faces." << std::endl;
        std::cout << "using a resolution of " << resolution << "mm/pixel" << std::endl;
        
        // PROCESS OUTPUT
        myIFS sphere = projection.exportToIFS(1000.0);
        IFS::exportOBJ(sphere, "sphere.obj");
        
        
        // TODO: 
        // - input E57 image to read in pose (position / orientation)
        // - model position and orientation of pano
        // for each face:
        //   for each pixel: perform ray intersection with pano
        //   write output image

        int faceid = 0;
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
                    oss << "ortho_" << faceid << ".pnm";
                    PNM::writePNM(orthophoto, oss.str());
                    std::cout << oss.str() << " : " << orthophoto.width() << "x" << orthophoto.height() << std::endl;
                }
            } 
            else
            {
                std::cout << "[ERR]: non-quad face detected (" << face.size() << " vertices)." << std::endl;
            }
            ++faceid;
        }

        return 0;
    }

    std::cout << "--help for options." << std::endl;

}