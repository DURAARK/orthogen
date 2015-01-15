
#include "projection.h"

#include <set>
#include <fstream>

// defaults to complete sphere
SphericalPanoramaImageProjection::SphericalPanoramaImageProjection()
    : azimuthRange(0, 2 * PI), elevationRange(-PI/2,PI/2), verbose(false)
{
}

void SphericalPanoramaImageProjection::setPanoramicImage(const Image &pi) 
{ 
    pano = pi; 
}

const Image & SphericalPanoramaImageProjection::img() const
{ 
    return pano; 
}

bool SphericalPanoramaImageProjection::isValid() const
{ 
    return pano.isValid(); 
}

void SphericalPanoramaImageProjection::setPosition(const Vec3d &pos)
{
    pose.O = pos;
}

bool SphericalPanoramaImageProjection::applyRotation(Quaterniond &quat)
{
    //std::cout << "CCS before rotation: " << std::endl << pose;
    pose.applyRotation(quat);
    //std::cout << "CCS after rotation: " << std::endl << pose;
    return true;
}


// Coordinate system conversions

// spherical to texture coordinate system
Vec2d SphericalPanoramaImageProjection::spher2tex(const Vec3d &spherical) const
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
    
// spherical to cartesian
Vec3d SphericalPanoramaImageProjection::spher2cart(const Vec3d &spc) const
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
Vec3d SphericalPanoramaImageProjection::cart2spher(const Vec3d &cart) const
{
    double radius = cart.norm();
    double elevation = PI/2 - acos(cart[2] / 
              sqrt(cart[0]*cart[0] + cart[1]*cart[1] + cart[2]*cart[2]));
    double azimuth = atan2(cart[1], cart[0]);
    if (azimuth < 0) azimuth += 2.0*PI;

    assert(elevation >= -PI/2 && elevation <= PI/2);
    assert(azimuth >= 0.0 && azimuth <= 2.0*PI);

    return Vec3d(azimuth, elevation, radius);

}

// world coordinate to texture coordinate
Vec2d SphericalPanoramaImageProjection::world2texture(const Vec3d &worldpos) const
{
    Eigen::Vector4d campos = pose.world2pose() * 
            Eigen::Vector4d(worldpos[0], worldpos[1], worldpos[2], 1.0);
    // normalize vector to get the intersection on the unit sphere
    Vec3d ray = Vec3d(campos[0]/campos[3], campos[1]/campos[3], campos[2]/campos[3]); 
    ray.normalize();
    Vec3d spherical = cart2spher(ray);
    return spher2tex(spherical);
}

// Projection
RGB SphericalPanoramaImageProjection::getColorProjection(const Vec3d &pos) const
{
    // transform point into camera coordinate system
    Vec2d texc = world2texture(pos);
    assert(texc[0] >= 0.0 && texc[0] <= 1.0 && texc[1] >= 0.0 && texc[1] <= 1.0);

    // pano lookup, nearest neighbor for now.. TODO: interpolation
    RGB pixel(0,0,0);

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
    return pixel;
}

// export to textured sphere
myIFS SphericalPanoramaImageProjection::exportTexturedSphere(const double r, const int numpts)
{
    myIFS result;
    result.useTextureCoordinates = true;

    const double elevationStep = (PI / numpts);
    const double azimuthStep = (2.0*PI / numpts);

    // CCW
    const Vec3d delta1(0, elevationStep, 0);
    const Vec3d delta2(azimuthStep, elevationStep, 0);
    const Vec3d delta3(azimuthStep, 0, 0);

    const Mat4 transform = pose.pose2world();
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
            myIFS::IFSINDICES ifsface;
            myIFS::IFSINDICES ifsfacetc;

            auto insertVertex = [ &ifsface, &ifsfacetc, &result](const Vec3d &vertex, const Vec2d &texc)
            {
                // get vertex index
                myIFS::IFSINDEX vi = result.vertex2index(vertex);
                ifsface.push_back(vi);
                // get texture coordinate index
                result.texcoordinates.push_back(texc);
                ifsfacetc.push_back(result.texcoordinates.size() - 1);
            };

            auto cartesianVertex = [&transform, this](const Vec3d &spherical) -> Vec3d
            {
                const Vec3d ccspos = spher2cart(spherical);
                const Vec4d pos = transform * Vec4d(ccspos[0], ccspos[1], ccspos[2], 1.0);
                return Vec3d(pos[0] / pos[3], pos[1] / pos[3], pos[2] / pos[3]);
            };

            // for OBJ export: bottom left origin -> v = 1.0 - v
            insertVertex(cartesianVertex(spherical), spher2tex(spherical));
            insertVertex(cartesianVertex(spherical + delta1), spher2tex(spherical + delta1));
            insertVertex(cartesianVertex(spherical + delta2), spher2tex(spherical + delta2));
            insertVertex(cartesianVertex(spherical + delta3), spher2tex(spherical + delta3));
            // process only non-degenerated faces
            if (ifsface.size() == 4)
            {
                result.faces.push_back(ifsface);
                result.facetexc.push_back(ifsfacetc);
            }
        }
    }
    return result;
}

void SphericalPanoramaImageProjection::exportPointCloud(const double r, const double numpts)
{
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
}
