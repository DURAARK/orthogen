#ifndef _QUAD_H_
#define _QUAD_H_

#include "projection.h"

// Quad3D : an oriented quad in 3D space.
// The class receives the 4 coordinates in CCW order and
// arranges the vertices such that the first coordinate is upper left.
// ulrich.krispel@fraunhofer.at

template< class T >
struct Quad3D
{
    T V[4];                        // quad coordinates, in CCW Order starting top left
    Pose pose;
    //int firstVertex;

    std::vector<size_t> tri_id;    // mesh triangles belonging to this quad

    // vertices are given in CCW order
    Quad3D()
    {
    }

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
        int firstVertex = brak[3].second;
        switch (firstVertex)
        {
            case 0: V[0] = v0; V[1] = v1; V[2] = v2; V[3] = v3; break;
            case 1: V[0] = v1; V[1] = v2; V[2] = v3; V[3] = v0; break;
            case 2: V[0] = v2; V[1] = v3; V[2] = v0; V[3] = v1; break;
            case 3: V[0] = v3; V[1] = v0; V[2] = v1; V[3] = v2; break;
            default:
            std::abort();
        }
        calculatePose();
    }

    void calculatePose()
    {
        // V1 is origin, bottom left
        T X = V[2] - V[1];
        X.normalize();
        T Y = V[0] - V[1];
        Y.normalize();
        T Z = X.cross(Y);
        pose = Pose(V[1], X, Y, Z);
    }

    // project any 3d point into the quad plane using normal projection,
    // and calculate texture coordinates
    inline Vec2d point2tex(const Vec3d &p) const
    {
        Vec3d qpos = pose.world2pose(p);
        Vec3d wh = pose.world2pose(V[3]);        // get width and height
        return Vec2d(qpos[0] / wh[0], qpos[1] / wh[1]);
    }

    // assert that v0..v3 actually form a rectangle.
    inline double area() 
    {
        return (V[1] - V[0]).norm() * (V[2] - V[1]).norm();
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

#endif
