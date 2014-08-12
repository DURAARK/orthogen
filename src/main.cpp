
#include "image.h"
#include "pnm.h"
#include "ifs.h"
#include "vec3.h"

#include <vector>
#include <string>

#include <cmath>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

typedef IFS::IFS< Vec3d > myIFS;

typedef MathVector<unsigned char,3> RGB;    // RGB color value
#define PI 3.141592653589793238462643

// orthogonal right handed coordinate system
template <class V>
struct RHCS      
{
private:
    V x;
    V y;
    V z;
public:
    // default: x right, y up, z negative view direction
    RHCS() : x(1.0, 0.0, 0.0), y(0.0, 1.0, 0.0), z(0.0, 0.0, -1.0) {}

    inline void setPose(const V &xx, const V &yy, const V &zz) { x = xx; y = yy; z = zz; }
    inline V getRight() const { return x; }
    inline V getUp()    const { return y; }
    inline V getFron()  const { return z; }
};

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
    Vec3d C;        // camera center
    RHCS<Vec3d> cs; // orientation (coordinate system)
    Image pano;
public:

    SphericalPanoramaImageProjection() : C(0,0,0)
    {

    }

    inline void setPanoramicImage(const Image &pi) { pano = pi; }
    inline void setCenter(const Vec3d &center)     { C = center; }
    inline void setPose(const Vec3d &right, const Vec3d &up, const Vec3d &front) { }

    inline const Image & img() { return pano; }

    inline bool isValid() const { return pano.isValid(); }
    Vec3d getPosition() { return C; }
    RGB   getColorProjection(const Vec3d &pos) const
    {
        // get vector from camera center to position
        Vec3d ray = pos - C;

        // normalize vector to get the intersection on the unit sphere
        ray.normalize();

        // cartesian to spherical coordinates
        // pano-x = phi = atan2(y,x) normalized to [0,1]
        // pano-y = theta = arccos(z/sqrt(x^2+y^2+z^2)) normalized to [-1,1], norm can be neglected 
        //                                         since (x,y,z) is already normalized
        double phi = -atan2(ray[1], ray[0]);
        if (phi < 0) phi += 2.0*PI;
        phi /= 2.0*PI;

        double theta = acos(ray[2]);
        if (theta < 0) theta += 2.0*PI;
        theta /= 2.0*PI;
        
        // pano lookup
        RGB pixel;
        pixel[0] = pano(phi*pano.width(), theta*pano.height(), 0);
        pixel[1] = pano(phi*pano.width(), theta*pano.height(), 1);
        pixel[2] = pano(phi*pano.width(), theta*pano.height(), 2);
        return pixel;
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
                Image orthophoto = quad.performProjection(projection, 1.0);
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