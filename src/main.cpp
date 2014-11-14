

#include <vector>
#include <string>

#include <cmath>

// use eigen matrices
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ifs.h"
#include "image.h"
#include "pnm.h"

#include "meanshift.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

typedef Eigen::Quaternion<double> Quaterniond;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector2d Vec2d;

typedef Eigen::Matrix< unsigned char, 3, 1 > RGB;
typedef Eigen::Vector4i Vec4i;


typedef IFS::IFS< Vec3d, Vec2d > myIFS;

#define PI 3.141592653589793238462643
#define PIf 3.141592653589793238462643f

//#define NEAREST_NEIGHBOR

struct IPlane
{
    int x[4];
    inline IPlane(const int a, const int b, const int c, const int d) 
    {
        x[0] = a; x[1] = b; x[2] = c; x[3] = d;
    }

    inline int &operator[](const unsigned int index) { return x[index]; }
    inline const bool operator<(const IPlane &other) const
    {
        if (x[0] < other.x[0]) return true;
        if (x[0] > other.x[0]) return false;
        if (x[1] < other.x[1]) return true;
        if (x[1] > other.x[1]) return false;
        if (x[2] < other.x[2]) return true;
        return false;
    }
};
// support for ostream
std::ostream& operator<< (std::ostream &os, const IPlane &p)
{
    // write inequality
    os << "[" << p.x[0] << "," << p.x[1] << "," << p.x[2] << "|" << p.x[3] << "]";
    return os;
};


template <typename VTYPE>
inline double vnorm(VTYPE &v) 
{
    double norm = 0;
    for (int i = 0; i < VTYPE::RowsAtCompileTime; ++i)
    {
        norm += v[i] * v[i];
    }
    return sqrt(norm);
}


// -------------------------------------------------------
// greatest common divisor of two and three elements
// use GCD2,3,4

template< class T >
inline static T _gcd2(T u, T v)
{
    assert(u >= 0);
    assert(v >= 0);
    if (u == v) return u;
    if (u == 0) return v;
    if (v == 0) return u;
    if (~u & 1)
    {
        if (v & 1) { return _gcd2(u >> 1, v); }
        else       { return _gcd2(u >> 1, v >> 1) << 1; }
    }
    if (~v & 1)   { return _gcd2(u, v >> 1); }
    if (u > v)    { return _gcd2((u - v) >> 1, v); }
    return _gcd2((v - u) >> 1, u);
}
template< class T >
inline static T GCD2(T u, T v) {
    return _gcd2(std::abs(u), std::abs(v));
}

template< class T >
inline static T _gcd3(T u, T v, T w) {
    return _gcd2<T>(_gcd2<T>(u, v), w);
}

template< class T >
inline static T GCD3(T u, T v, T w) {
    return _gcd3(std::abs(u), std::abs(v), std::abs(w));
}

template< class T >
inline static T _gcd4(T u, T v, T w, T x) {
    return _gcd2<T>(_gcd2<T>(u, v), _gcd2<T>(w, x));
}
template< class T >
inline static T GCD4(T u, T v, T w, T x) {
    return _gcd4(std::abs(u), std::abs(v), std::abs(w), std::abs(x));
}



// e57 compatible camera pose 
struct Pose
{
    Vec3d O;
    Vec3d X;
    Vec3d Y;
    Vec3d Z;

    // scanner coordinate system is X-right, Y-front, Z-up
    Pose() : O(0, 0, 0), X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1)
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
        Vec2d tc(
            ((spherical[0] - azimuthRange[0]) / (azimuthRange[1] - azimuthRange[0])),
            ((elevationRange[1] - spherical[1]) / (elevationRange[1] - elevationRange[0]))
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
    Vec3d spher2cart(const Vec3d &spc) const
    {
        return Vec3d(
            spc[2] * sin(PI/2-spc[1]) * cos(spc[0]),
            spc[2] * sin(PI/2-spc[1]) * sin(spc[0]),
            spc[2] * cos(PI/2-spc[1])
            );
    }

    // cartesian to spherical coordinates
    Vec3d cart2spher(const Vec3d &cart) const
    {
        double radius = vnorm(cart);
        //double elevation = PI/2 - acos(cart[2] / radius);
        double elevation = atan2(cart[2], sqrt(cart[0] * cart[0] + cart[1] * cart[1]));
        double azimuth = atan2(cart[1], cart[0]);
        if (azimuth < 0) azimuth += 2.0*PI;

        assert(elevation >= -PI/2 && elevation <= PI/2);
        assert(azimuth >= 0.0 && azimuth <= 2.0*PI);

        return Vec3d(azimuth, elevation, radius);

    }


    Vec2d world2texture(const Vec3d &worldpos) const
    {
        auto campos = pose.world2pose() * Eigen::Vector4d(worldpos[0],worldpos[1],worldpos[2],1.0);
        // normalize vector to get the intersection on the unit sphere
        Vec3d ray = Vec3d(campos[0] / campos[3], campos[1] / campos[3], campos[2] / campos[3]); ray.normalize();
        Vec3d spherical = cart2spher(ray);
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
            Vec3d pix = pano.bilinear<Vec3d>(texc[0] * pano.width(), texc[1] * pano.height());
            pixel[0] = (unsigned char)pix[0];
            pixel[1] = (unsigned char)pix[1];
            pixel[2] = (unsigned char)pix[2];
#endif
        }
        return pixel;
    }
  
    // export to textured sphere
    myIFS exportTexturedSphere(const double r = 1.0, const double numpts = 100)
    {
        myIFS result;
        result.useTextureCoordinates = true;

        const double elevationStep = (PI / numpts);
        const double azimuthStep = (2.0*PI / numpts);


        const Vec3d delta1(azimuthStep, 0, 0);
        const Vec3d delta2(azimuthStep, elevationStep, 0);
        const Vec3d delta3(0, elevationStep, 0);
        const Vec2d OBJ_V_ADJ(0, 1.0);

        std::vector<Vec3d> point;
        std::vector<Vec3d> color;

        const Mat4 transform = pose.pose2world();

        for (double e = -PI / 2; e <= PI / 2; e += elevationStep)
        {
            for (double a = 0; a <= 2 * PI; a += azimuthStep)
            {
                const Vec3d spherical(a, e, r);
#ifdef _DEBUG
                const Vec3d cart = spher2cart(spherical);
                const Vec3d sph2 = cart2spher(cart);
                assert((spherical - sph2).stableNorm() < 0.01); // validate conversion
                const Vec2d tex = spher2tex(spherical);
                //std::cout << " Spherical : azimuth " << spherical[0] << " elevation " << spherical[1] << " radius " << spherical[2] << std::endl;
                //std::cout << " Texture   : X " << tex[0] << " Y " << tex[1] << std::endl;
#endif


                std::set<myIFS::IFSINDEX> face;
                myIFS::IFSFACE ifsface;

                auto insertVertex = [&face, &ifsface](myIFS::IFSINDEX i)
                {
                    face.insert(i);
                    ifsface.push_back(i);
                };

                auto cartesianVertex = [&transform, this](const Vec3d &spherical) -> Vec3d
                {
                    const Vec3d ccspos = spher2cart(spherical);
                    const Vec4d pos = transform * Vec4d(ccspos[0], ccspos[1], ccspos[2], 1.0);
                    return Vec3d(pos[0] / pos[3], pos[1] / pos[3], pos[2] / pos[3]);
                };

                // for OBJ export: bottom left origin -> v = 1.0 - v


                insertVertex(result.vertex2index(cartesianVertex(spherical), OBJ_V_ADJ - spher2tex(spherical)));
                insertVertex(result.vertex2index(cartesianVertex(spherical + delta1), OBJ_V_ADJ - spher2tex(spherical + delta1)));
                insertVertex(result.vertex2index(cartesianVertex(spherical + delta2), OBJ_V_ADJ - spher2tex(spherical + delta2)));
                insertVertex(result.vertex2index(cartesianVertex(spherical + delta3), OBJ_V_ADJ - spher2tex(spherical + delta3)));
                // process only non-degenerated faces
                if (face.size() == 4)
                {
                    result.faces.push_back(ifsface);
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

        // create points on a sphere
        for (double e = -PI/2; e <= PI/2; e += elevationStep)
        {
            for (double a = 0; a <= 2 * PI; a += azimuthStep)
            {
                const Vec3d spherical(a, e, r);
                assert(vnorm(cart2spher(spher2cart(spherical)) - spherical) <= 0.01);
                const Vec3d ccspos = spher2cart(spherical);
                const Vec4d pos  = modelview * Vec4d(ccspos[0],ccspos[1],ccspos[2],1.0);
                const Vec3d wpos = Vec3d(pos[0] / pos[3], pos[1] / pos[3], pos[2] / pos[3]);
                RGB pcol = getColorProjection(wpos);
                {
                    point.push_back(wpos);
                    color.push_back(Vec3d(pcol[0], pcol[1], pcol[2]) / 255.0);
                }

            }

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
        T W  = V[1] - V[0];
        T H = V[3] - V[0];

        double width = vnorm(W);
        double height = vnorm(H);

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

// extract quads from indexed face set

std::vector<Quad3Dd> extractQuads(const myIFS &ifs)
{
    // perform clustering of vertices into face planes
    std::cout << "performing quad extraction..." << std::endl;
    // canonical plane representation:  a * x + b * y + c * z + d = 0
    // => d = -<n,p>
    // [ a b c d ] with a,b,c,d being integers and gcd(a,b,c,d) = 1

    std::vector<Quad3Dd> result;

    struct DetectedPlane
    {
        Pose ccs;						// reference coordinate system
        std::set<myIFS::IFSINDEX> vi;	// vertex indices
    };


    std::map<IPlane, DetectedPlane> planemap;       // map vertices to planes  LexicographicalComparator<Vec4i>

    // STEP 1: Clustering
    int faceid = 0;
    for (auto const &face : ifs.faces)
    {
        if (face.size() >= 3)
        {
            // calculate normal from face
            Vec3d n = (ifs[face[1]] - ifs[face[0]]).cross(ifs[face[2]] - ifs[face[0]]);
            //CrossProduct(ifs[face[1]] - ifs[face[0]], ifs[face[2]]-ifs[face[0]]);
            n.normalize();
            // quantize
            int a, b, c, d;
            a = (int)round(n[0] * 5.0);
            b = (int)round(n[1] * 5.0);
            c = (int)round(n[2] * 5.0);
            // calculate d: distance from origin.. maybe scale by resolution?
            d = (int)round(n.dot(ifs[face[0]])/10.0);

            // TODO: maybe linear regression of the plane

            int gcd = GCD3(a, b, c);
            IPlane plane(a / gcd, b / gcd, c / gcd, d);

            std::cout << "face " << faceid << plane << std::endl;

            // insert vertices into plane
            if (planemap.find(plane) == planemap.end())
            {
                planemap[plane].ccs.Z = n;  // update normal
            }
            else 
            {
                // should be similar, quite ad-hoc test here
                const double d = vnorm(planemap[plane].ccs.Z - n);
                assert(d < 1.0);
            }
            for (auto const &vindex : face)
                planemap[plane].vi.insert(vindex);
        }
        ++faceid;
    }

    // STEP 2: calculate principal axes of plane vertices and create bounding quads
    std::cout << "processing " << planemap.size() << " quads.." << std::endl;
    for (auto &it : planemap)
    {
        // calculate PCA
        Vec3d p_(0, 0, 0);          // mean coordinate vector
        //std::cout << "# COORDINATES" << std::endl;
        for (auto const &vindex : it.second.vi)
        {
            //std::cout << ifs[vindex] << std::endl;
            p_ += ifs[vindex];
        }
        p_ /= it.second.vi.size();
        //std::cout << "# MEAN: " << p_ << std::endl;

        // calculate covariance matrix
        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        for (int m = 0; m < 3; ++m)
        {
            for (int n = 0; n < 3; ++n)
            {
                for (auto const &i : it.second.vi)
                {
                    M(m, n) += (ifs[i][m] - p_[m]) * (ifs[i][n] - p_[n]);
                }
                M(m, n) /= it.second.vi.size();
            }
        }

        //std::cout << "#### FACE:" << std::endl;
        //std::cout << " Pose:" << it.second.ccs << "   plane equation:" << it.first << std::endl;
//        std::cout << "M:" << std::endl;
//        std::cout << M << std::endl;;
        // calculate eigenvalues
        // This matrix is real and symmetric, 
        // therefore we can use SelfAdjointEigenSolver
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
        es.compute(M);
//        std::cout << "The eigenvalues of M are: " << std::endl << es.eigenvalues().transpose() << std::endl;
//        std::cout << "The eigenvectors of M are: " << std::endl << es.eigenvectors() << std::endl;

        // sort eigenvalues
        typedef std::pair<double, unsigned int> SEigVal;
        std::map<double, unsigned int> sortedEigValue;
        sortedEigValue.insert(SEigVal(-es.eigenvalues()[0], 0));
        sortedEigValue.insert(SEigVal(-es.eigenvalues()[1], 1));
        sortedEigValue.insert(SEigVal(-es.eigenvalues()[2], 2));

        //for (auto const it : sortedEigValue)
        //{
        //    std::cout << "eigenvalue:" << it.first << "  eigenvector: " << es.eigenvectors().col(it.second).transpose() << std::endl;
        //}

        // get principal axis
        auto getEigenVector = [&es](const SEigVal &ev) -> Vec3d
        {
            auto evec = es.eigenvectors().col(ev.second);
            return Vec3d(evec[0], evec[1], evec[2]);
        };

        auto iteg = sortedEigValue.begin();
        Vec3d PA1 = getEigenVector(*iteg);
        //std::cout << "Prinxipal Axis 1: " << PA1 << std::endl;
        ++iteg;
        Vec3d PA2 = getEigenVector(*iteg);
        //std::cout << "Prinxipal Axis 2: " << PA2 << std::endl;
        ++iteg;
        Vec3d PA3 = getEigenVector(*iteg);
        //std::cout << "Prinxipal Axis 3: " << PA3 << std::endl;

        // determine X and Y direction from PA1 and PA2
        // Assumption: (0,0,1) in world coordinates is upvector 
        // -> Y direction is the vector with bigger absolute z value
        if (std::abs(PA1[2]) < std::abs(PA2[2]))
        {
            it.second.ccs.X = PA1;
            it.second.ccs.Y = PA2;
        }
        else 
        {
            it.second.ccs.X = PA2;
            it.second.ccs.Y = PA1;
        }

        // rotate such that y is facing downwards (image coordinates)
        // X should therefore face to the right, if the normal pointed away
        // from the origin
        if (it.second.ccs.Y[2] > 0)
        {
            // rotate by 180°
            it.second.ccs.X *= -1.0;
            it.second.ccs.Y *= -1.0;
        }

        //std::cout << "Plane X-Direction : " << it.second.ccs.X << std::endl;
        //std::cout << "Plane Y-Direction : " << it.second.ccs.Y << std::endl;
        // plane normal vector is given, calculate distance from origin
        const double planed = -(it.second.ccs.Z.dot(ifs[*it.second.vi.begin()]));
        // create a origin in the plane coordinate system
        it.second.ccs.O = it.second.ccs.Z * -planed;


        //// find maximum bounds in X and Y, sort by projection along the axis
        //std::map< double, myIFS::IFSINDEX> sortX;
        //std::map< double, myIFS::IFSINDEX> sortY;
        //for (auto const &vindex : it.second.vi)
        //{
        //    sortX[ifs[vindex].dot(it.second.ccs.X)] = vindex;
        //    sortY[ifs[vindex].dot(it.second.ccs.Y)] = vindex;
        //}
        //// get maximum bounds
        //std::cout << "SortX:";
        //for (auto const itx : sortX) { std::cout << ifs[itx.second].transpose() << ", "; }
        //std::cout << std::endl;
        //std::cout << "SortY:";
        //for (auto const ity : sortY) { std::cout << ifs[ity.second].transpose() << ", "; }


        //// test if plane origin and face points lie in the plane
        //auto testInPlane = [](const Vec3d &n, const double d, const Vec3d &p)
        //{
        //	const double off = (n.dot(p)) + d;
        //	assert(std::abs(off) < 0.01);
        //};
        //testInPlane(it.second.ccs.Z, planed, it.second.ccs.O);
        //for (auto const &vindex : it.second.vi)
        //{
        //	testInPlane(it.second.ccs.Z, planed, ifs[vindex]);
        //}

        //std::cout << " POSE: " << it.second.ccs << std::endl;

        // calculate quad bounds in the plane coordinate system with
        const Mat4 T = it.second.ccs.world2pose();
        Vec2d xrange(DBL_MAX, DBL_MIN);
        Vec2d yrange(DBL_MAX, DBL_MIN);
        for (auto const &vindex : it.second.vi)
        {
            // project point to plane coordinate system
            Vec4d wh(ifs[vindex][0], ifs[vindex][1], ifs[vindex][2], 1.0);
            Vec4d p = T * wh;
            if (p[0] < xrange[0]) xrange[0] = p[0];
            if (p[0] > xrange[1]) xrange[1] = p[0];
            if (p[1] < yrange[0]) yrange[0] = p[1];
            if (p[1] > yrange[1]) yrange[1] = p[1];
        }

        // create quad
        const Mat4 WT = it.second.ccs.pose2world();
        auto projMul = [&WT](const Vec3d &v) -> Vec3d
        {
            Vec4d r = WT * Vec4d(v[0], v[1], v[2], 1.0);
            return Vec3d(r[0] / r[3], r[1] / r[3], r[2] / r[3]);
        };

        result.push_back(Quad3Dd(
            projMul(Vec3d(xrange[0], yrange[0], 0.0)),
            projMul(Vec3d(xrange[0], yrange[1], 0.0)),
            projMul(Vec3d(xrange[1], yrange[1], 0.0)),
            projMul(Vec3d(xrange[1], yrange[0], 0.0))
            ));
    }
    return result;
}



int main(int ac, char* av[])
{
    SphericalPanoramaImageProjection projection;
    myIFS ingeometry;
    double resolution = 10.0;        // default: 1 mm/pixel
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
            ("exgeom", po::value< bool >(), "export geometry")
            ("exsphere", po::value< bool >(), "export panoramic sphere")
            ;

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
            std::cout << "- exporting projected panorama OBJ.." << std::endl;
            myIFS sphere = projection.exportTexturedSphere(1.0, 100);
            sphere.facematerial[0] = IFS::Material("sphere", "sphere.jpg");
            IFS::exportOBJ(sphere, "sphere", "# OrthoGen panoramic sphere\n");
            // TODO: also export jpg

        }

        //std::cout << "- exporting panorama pointcloud WRL.." << std::endl;
        //projection.exportPointCloud(1.0, 100);

        //
        // TODO: 
        // - input E57 image to read in pose (position / orientation)
        // - model position and orientation of pano
        // for each face:
        //   for each pixel: perform ray intersection with pano
        //   write output image

        //// PROJECT POINTS INTO PANO
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
                {
                    std::ostringstream matname, texname;
                    matname << "ortho" << faceid;
                    texname << "ortho_" << faceid << ".jpg";
                    outgeometry.facematerial[faceid] = IFS::Material(matname.str(), texname.str());
                    // push texture coordinates
                    outgeometry.texcoordinates.push_back(Vec2d(0, 0));
                    outgeometry.texcoordinates.push_back(Vec2d(0, 1));
                    outgeometry.texcoordinates.push_back(Vec2d(1, 1));
                    outgeometry.texcoordinates.push_back(Vec2d(1, 0));
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