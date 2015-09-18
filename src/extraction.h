#ifndef _EXTRACTION_H_
#define _EXTRACTION_H_

#include "quad.h"

namespace OrthoGen
{
    typedef Quad3D<Vec3d> Quad3Dd;

    struct Triangle
    {
        const Vec3d p, q, r;      // vertices
        const Vec3d m;            // mean (midpoint)
        const Vec3d n;            // normal
        const double area;
        const size_t faceid;      // triangle id 
        int cluster;              // quad id



#define _TA (Q-P)
#define _TB (R-P)
#define _C1_ (_TA[1] * _TB[2] - _TA[2] * _TB[1])
#define _C2_ (_TA[2] * _TB[0] - _TA[0] * _TB[2])
#define _C3_ (_TA[0] * _TB[1] - _TA[1] * _TB[0])

        Triangle(const Vec3d &P, const Vec3d &Q, const Vec3d &R, size_t face)
            : p(P), q(Q), r(R), m((P + Q + R) / 3.0),
            n(_TA.cross(_TB).normalized()), area(0.5*sqrt(_C1_*_C1_ + _C2_*_C2_ + _C3_*_C3_)),
            faceid(face), cluster(-1)
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

        bool inside(const Vec3d &p) const
        {
            return (p[0] >= bbmin[0] && p[0] <= bbmax[0] &&
                p[1] >= bbmin[1] && p[1] <= bbmax[1] &&
                p[2] >= bbmin[2] && p[2] <= bbmax[2]);
        }

        double volume() const
        {
            return (bbmax[0] - bbmin[0])*(bbmax[1] - bbmin[1])*(bbmax[2] - bbmin[2]);
        }
    };

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

    // quad patch extraction with clustering
    void extract_quads(const myIFS &ifs, const double scalef,
        std::vector<Triangle> &triangles,
        std::vector<Quad3Dd> &quads,
        double WINDOW_SIZE_DIST = 0.1, double WINDOW_SIZE_NORML = 0.3)
    {
        std::cout.precision(2);

        // extract triangles from faces
        {
            int numtri = 0, numquad = 0, numIgnored = 0;
            for (auto &face : ifs.faces)
            {
                switch (face.size())
                {
                case 3: // Triangle
                    triangles.push_back(Triangle(ifs.vertices[face[0]],
                        ifs.vertices[face[1]],
                        ifs.vertices[face[2]], triangles.size()));
                    ++numtri;
                    break;
                case 4: // Quad
                    triangles.push_back(Triangle(ifs.vertices[face[0]],
                        ifs.vertices[face[1]],
                        ifs.vertices[face[2]], triangles.size()));

                    triangles.push_back(Triangle(ifs.vertices[face[2]],
                        ifs.vertices[face[3]],
                        ifs.vertices[face[0]], triangles.size()));
                    ++numquad;
                    break;

                default:
                    std::cout << "[Warning] face with " << face.size()
                        << " vertices ignored." << std::endl;
                    ++numIgnored;

                }
            }
            std::cout << "Input Geometry consists of " << numtri << " triangles and " << numquad << " quads." << std::endl;
            if (numIgnored > 0) {
                std::cout << numIgnored << " elements ignored." << std::endl;
            }
        }

        // cluster normals
        MEANSHIFT::Meanshift<Vec3d, 3> ms_normals;
        for (auto const &t : triangles)
        {
            ms_normals.points.push_back(t.n);
        }
        ms_normals.calculate(WINDOW_SIZE_NORML);

        // combine clusters by implicit plane similarity: 
        // perform mean shift on cluster main direction angles
        std::cout << "found " << ms_normals.cluster.size() << " normal clusters." << std::endl;


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
                row[i++] = (s > MS_ANGLE_COS) ? 1.0 : 0.0;
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
                        for (size_t id : ncluster->second)
                        {
                            tricluster.push_back(triangles[id]);
                        }
                    }
                }
            }

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
            ms_distance.calculate(scalef * WINDOW_SIZE_DIST);  // default window size = 10cm

            std::cout << " clusters: " << ms_distance.cluster.size() << std::endl;

            for (auto const &dcluster : ms_distance.cluster)
            {
                Quad3Dd Q;
                // get centroid of cluster
                Vec3d center = Vec3d::Zero();
                for (const size_t i : dcluster.second) { center += tricluster[i].m; }
                center /= (double)dcluster.second.size();

                // create a local coordinate system, Z = clusterdirection
                Vec3d X, Y, Z, T;

                // create orthonormal basis from clusterdirection
                // http://orbit.dtu.dk/files/57573287/onb_frisvad_jgt2012.pdf
                Z = clusterdirection;
                if (Z[2] < -0.999999f)
                {
                    X << 0.0, -1.0, 0.0;
                    Y << -1.0, 0.0, 0.0;
                }
                else
                {
                    const double a = 1.0 / (1.0 + Z[2]);
                    const double b = -Z[0] * Z[1] * a;
                    X << (1.0 - Z[0] * Z[0] * a), b, -Z[0];
                    Y << b, (1.0 - Z[1] * Z[1] * a), -Z[1];
                }

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
                    for (const size_t i : dcluster.second)
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

};

#endif
