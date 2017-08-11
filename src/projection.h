#ifndef _PROJECTION_H_
#define _PROJECTION_H_

// image based projections
// ulrich.krispel@fraunhofer.at

#include "types.h"
#include "pose.h"
#include "image.h"

namespace OrthoGen
{

    // ImageProjection Interface
    class ImageProjection
    {
    public:
        virtual RGB getColorProjection(const Vec3d &pos) const = 0;
    };


    // Spherical Panorama Image Projection

    class SphericalPanoramaImageProjection : public ImageProjection
    {
    private:
        bool verbose;

    public:
        std::string basename;
        int         id;
        Vec2d azimuthRange;
        Vec2d elevationRange;

        Pose pose;
        Image pano;
        std::string panofile;

        SphericalPanoramaImageProjection();

        bool isValid() const;

        void setPanoramicImage(const std::string &file, const Image &pi);
        const Image & img() const;

        void setPosition(const Vec3d &pos);
        bool applyRotation(const Quaterniond &quat);

        void printPose();

        // COORDINATE SYSTEM CONVERSIONS

        // spherical to texture coordinate system
        // spherical is in [azimuth | elevation | radius]
        Vec2d spher2tex(const Vec3d &spherical) const;

        // cartesian coordinates: Vec3 [ x, y, z ]
        // spherical coordinates: Vec3 [ azimuth , elevation , radius ]
        // azimuth: 0..2*pi, elevation: -PI/2..+PI/2, radius>0
        Vec3d spher2cart(const Vec3d &spc) const;

        // cartesian to spherical coordinates
        Vec3d cart2spher(const Vec3d &cart) const;

        // world coordinate to texture coordinate
        Vec2d world2texture(const Vec3d &worldpos) const;

        RGB   getColorProjection(const Vec3d &pos) const;

        // export to textured sphere
        myIFS exportTexturedSphere(const double r = 1.0, const int numpts = 100);
        // export to wrl pointcloud
        void exportPointCloud(const double r = 1.0, const double numpts = 100);
    };

};

#endif
