#ifndef _POSE_H_
#define _POSE_H_

// e57 compatible camera pose 
// ulrich.krispel@fraunhofer.at


namespace OrthoGen {

    struct Pose
    {
        OrthoGen::Vec3d O;
        OrthoGen::Vec3d X;
        OrthoGen::Vec3d Y;
        OrthoGen::Vec3d Z;

        inline void reset()
        {
            O = Vec3d(0, 0, 0);
            X = Vec3d(-1, 0, 0);
            Y = Vec3d(0, 1, 0);
            Z = Vec3d(0, 0, 1);
        }

        // scanner coordinate system is X-right, Y-front, Z-up
        inline Pose()
        {
            reset();
        }
        inline Pose(const Vec3d &o, const Vec3d &x, const Vec3d &y, const Vec3d &z)
            : O(o), X(x), Y(y), Z(z)
        {
        }

        inline void applyRotation(const Quaterniond &q)
        {
            X = q * X;
            Y = q * Y;
            Z = q * Z;
        }

        // std output stream support
        inline friend std::ostream& operator<<(std::ostream& os, const Pose& c)
        {
            os << "O: " << c.O.transpose() << std::endl <<
                " X:" << c.X.transpose() << std::endl <<
                " Y:" << c.Y.transpose() << std::endl <<
                " Z:" << c.Z.transpose() << std::endl;
            return os;
        }

        inline Mat4 world2pose() const
        {
            Mat4 M;
            M.col(0) = Vec4d(X[0], Y[0], Z[0], 0.0);
            M.col(1) = Vec4d(X[1], Y[1], Z[1], 0.0);
            M.col(2) = Vec4d(X[2], Y[2], Z[2], 0.0);
            M.col(3) = Vec4d(-X.dot(O), -Y.dot(O), -Z.dot(O), 1.0);
            return M;
        }

        inline Vec3d world2pose(const Vec3d &v) const
        {
            const Mat4 M = world2pose();
            Vec4d V(v[0], v[1], v[2], 1.0);
            Vec4d R = M*V;
            return Vec3d(R[0] / R[3], R[1] / R[3], R[2] / R[3]);
        }

        inline Mat4 pose2world() const
        {
            Mat4 M;
            M.col(0) = Vec4d(X[0], X[1], X[2], 0.0);
            M.col(1) = Vec4d(Y[0], Y[1], Y[2], 0.0);
            M.col(2) = Vec4d(Z[0], Z[1], Z[2], 0.0);
            M.col(3) = Vec4d(O[0], O[1], O[2], 1.0);
            return M;
        }

        inline Vec3d pose2world(const Vec3d &v) const
        {
            const Mat4 M = pose2world();
            Vec4d V(v[0], v[1], v[2], 1.0);
            Vec4d R = M*V;
            return Vec3d(R[0] / R[3], R[1] / R[3], R[2] / R[3]);
        }


    };

};

#endif
