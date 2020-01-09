
//
// ortho view generation tool
// ulrich.krispel@fraunhofer.at
//
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <chrono>

#include <cstdlib>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"   // FileReadStream
#include "rapidjson/encodedstream.h"    // AutoUTFInputStream

#include "types.h"

#include "meanshift.h"
#include "extraction.h"
#include "normenc.h"

#include "config.h"

// configuration variables


typedef struct json_token JSONToken;
typedef struct json_token *JSONTokens;

using namespace OrthoGen;

const std::string ORTHOGEN_VERSION = "0.10.3";

template <typename V>
void prvec(const V& v, const std::string msg="")
{
    std::cout << msg << "[" << v[0] << "," << v[1] << "," << v[2] << "]";
}

Config CONFIG;
State  STATE;

int main(int ac, char *av[]) {
  std::cout << std::fixed;
  std::cout << "OrthoGen orthographic image generator " << ORTHOGEN_VERSION << std::endl;
  std::cout << "developed by Fraunhofer Austria Research GmbH" << std::endl;
  std::cout << "--help for options." << std::endl;

  try {

    config_parse_options(ac, av, CONFIG);
  // Input specification:

  // per scan (read from json)
  //  - name [panorama=.jpg]
  //  - pose
  //  - read in panoramic images

    // TODO: integrate old method of e57metadata + wall json parsing
    config_parse_json(CONFIG, STATE);

	// apply geometry offset
	for (auto &v : STATE.ifs.vertices) {
		v += CONFIG.geometry_offset;
	}

	const boost::filesystem::path jsonpath(CONFIG.jsonconfig);
	const boost::filesystem::path outdir =
		jsonpath.parent_path() / CONFIG.outdir;
	if (!boost::filesystem::exists(outdir)) {
		boost::filesystem::create_directory(outdir);
	}

  // Orthogen will
  // (a) extract rectangular patches
  //     - from an OBJ file
    std::cout << "* extracting rectangular patches" << std::endl;
    extract_quads(STATE.ifs, STATE.triangles, STATE.patches, CONFIG.WINDOW_SIZE_DISTANCE, CONFIG.WINDOW_SIZE_NORMAL, CONFIG.WINDOW_SIZE_ANGLE);

    std::cout << "total patches extracted:" << STATE.patches.size() << std::endl;

    //
    if (CONFIG.exportClusters)
    {
		boost::filesystem::path outfile_cluster = outdir / (CONFIG.output + "_clusters.obj");

        std::cout << "* writing clustered triangles OBJ.." << std::endl;
        std::unordered_map< size_t, std::vector<size_t> > clusters;
        for (size_t i = 0, ie = STATE.triangles.size(); i < ie; ++i) {
            auto const &T = STATE.triangles[i];
            clusters[T.cluster].push_back(i);
        }
        std::ofstream of(outfile_cluster.c_str());
        of << std::endl;
        size_t voffset = 0;
        for (auto const &cluster : clusters) {
            myIFS ctri;
            of << "o cluster_" << cluster.first << std::endl;
            for (auto const &tri : cluster.second) {
                auto const &T = STATE.triangles[tri];
                const size_t i0 = ctri.vertex2index(T.p) + voffset;
                const size_t i1 = ctri.vertex2index(T.q) + voffset;
                const size_t i2 = ctri.vertex2index(T.r) + voffset;
                ctri.faces.push_back({ i0,i1,i2 });
            }
            IFS::exportOBJ(ctri, of, "");
            voffset += ctri.vertices.size();
            of << std::endl;
        }
    }


    // -------------------------------------------------- EXPORT ORTHO PHOTOS
    if (CONFIG.exportOrtho)
    {
        // export walls

        myIFS quadgeometry; // ifs with orthophoto quads

        quadgeometry.useTextureCoordinates = true;
        quadgeometry.texcoordinates.push_back(Vec2d(0, 1));
        quadgeometry.texcoordinates.push_back(Vec2d(0, 0));
        quadgeometry.texcoordinates.push_back(Vec2d(1, 0));
        quadgeometry.texcoordinates.push_back(Vec2d(1, 1));

        int patch_id = 0;
        for (int patch_id = 0; patch_id < STATE.patches.size(); ++patch_id) {
            //
            auto const &patch= STATE.patches[patch_id];

            std::ostringstream outfilebase;
            outfilebase << CONFIG.output << "_" << patch_id;
            
            std::cout << "* processing " << outfilebase.str() << "..." << std::endl;

            // perform orthophoto projection: calculate wall size

            Vec3d W = patch.W();
            Vec3d H = patch.H();
            const double width = W.norm();
            const double height = H.norm();
			const int pwidth = (int)(width / CONFIG.resolution) + 1;
			const int pheight = (int)(height / CONFIG.resolution) + 1;
            Vec3d xdir = W; xdir.normalize();
            Vec3d ydir = H; ydir.normalize();

            Image ortho;
            Image associatedScan;

			std::cout << "width: " << width << "m, height: " << height << "m. => pixel: " << pwidth << " x " << pheight << "." << std::endl;

            ortho.initialize(pwidth, pheight, 24);
            if (CONFIG.verbose) {
                prvec(patch.V[0], " V0:");
                prvec(patch.V[1], " V1:");
                std::cout << std::endl;
                prvec(patch.V[2], " V2:");
                prvec(patch.V[3], " V3:");
                std::cout << std::endl;
                std::cout << ortho.width() << "x" << ortho.height() << " n: " << patch.pose.Z << std::endl;
            }
            associatedScan.initialize(ortho.width(), ortho.height(), 8);

            // project each pixel
            for (int y = 0; y < ortho.height(); ++y)
            {
                std::cout << (y*100.0 / ortho.height()) << "%\r";
                const Vec3d vecy = H * (y / (double)ortho.height());
#pragma omp parallel for
                for (int x = 0; x < ortho.width(); ++x)
                {
                    // calculate position in world coordinates
                    const Vec3d vecx = W * (x / (double)ortho.width());
                    const Vec3d position = patch.V[0] + vecx + vecy;

                    // project color value: choose nearest scan, 
                    double neardist = DBL_MAX;
                    int scanid = -1;

                    for (int i = 0; i < STATE.scans.size(); ++i)
                    {
                        auto const &scan = STATE.scans[i];

                        // D = normalized direction from sphere center to patch point
                        Vec3d D = (position - scan.pose.O);
                        const double dist = D.norm();
                        D /= dist;

                        // OCCLUSION TEST
                        // consider only scans that are not occluded by other geometry
                        bool considerScan = true;
                        for (auto const &T : STATE.triangles) {
                            if (T.cluster != patch_id) {
                                double u, v, t;
                                if (ray_triangle_intersection(scan.pose.O, D,
                                    T.p, T.q, T.r, t, u, v)) {

                                    if (t > 0 && t < (dist-0.01)) {
                                        // occluder detected
                                        considerScan = false;
                                        break;
                                    }
                                }
                            }
                        }

                        if (considerScan) {
                            if (dist < neardist) {
                                neardist = dist;
                                scanid = i;
                            }
                        }
                    }   // for all scans
                    if (scanid != -1)
                    {
                        RGB color = STATE.scans[scanid].getColorProjection(position);
                        ortho.setRGB(x, y, color);
                        associatedScan(x, y, 0) = scanid;
                    }
                }
            }

            boost::filesystem::path outfile_jpg = outdir / (outfilebase.str() + ".jpg");
            saveJPEG(outfile_jpg.string().c_str(), ortho);

            if (CONFIG.exportQuad) {
                // create quad geometry, quads always use the same texcoordinates
                myIFS::IFSINDICES face;
                myIFS::IFSINDICES facetc;
                face.push_back(quadgeometry.vertex2index(patch.V[0]));
                facetc.push_back(0);
                face.push_back(quadgeometry.vertex2index(patch.V[1]));
                facetc.push_back(1);
                face.push_back(quadgeometry.vertex2index(patch.V[2]));
                facetc.push_back(2);
                face.push_back(quadgeometry.vertex2index(patch.V[3]));
                facetc.push_back(3);
                quadgeometry.faces.push_back(face);
                quadgeometry.facetexc.push_back(facetc);
                quadgeometry.materials.push_back(
                    IFS::Material(outfilebase.str(), outfilebase.str()+".jpg"));
                quadgeometry.facematerial.push_back(quadgeometry.materials.size() -
                    1);
                assert(quadgeometry.faces.size() == quadgeometry.facetexc.size()
                    && quadgeometry.faces.size() == quadgeometry.facematerial.size());
            }
        }
        if (CONFIG.exportQuad)
        {
			const boost::filesystem::path outfile_patches = outdir / (CONFIG.output + "_patches");
			std::cout << "writing patches..." << std::endl;
            IFS::exportOBJ(quadgeometry, outfile_patches.string(), "# OrthoGen textured quads\n");
        }

    }

    // project UV's back to triangles and export triangulation
    {
        myIFS exportIFS;
        exportIFS.useTextureCoordinates = true;
        for (auto const &T : STATE.triangles) {
            // export triangle
            const Vec3d v0 = T.p, 
                v1 = T.q, 
                v2 = T.r;
            exportIFS.faces.push_back({
                exportIFS.vertex2index(v0),
                exportIFS.vertex2index(v1),
                exportIFS.vertex2index(v2)
            });
            // generate texture coordinates
            if (T.cluster != -1) {
                auto const &Q = STATE.patches[T.cluster];
                // TODO: intersect quad with line from triangle vertex
                //       to center of associated scan
                const Vec2d tc0 = Q.point2tex(v0),
                    tc1 = Q.point2tex(v1),
                    tc2 = Q.point2tex(v2);

                size_t i = exportIFS.tc2index(tc0);

                exportIFS.facetexc.push_back({
                    exportIFS.tc2index(tc0),
                    exportIFS.tc2index(tc1),
                    exportIFS.tc2index(tc2)
                });
                // material: quad id
                exportIFS.facematerial.push_back(T.cluster);
            }
        }
        // set materials
        for (size_t i = 0; i < STATE.patches.size(); ++i) {
            std::ostringstream matname;
            matname << CONFIG.output << "_" << i;
            exportIFS.materials.push_back(
                IFS::Material(matname.str(), matname.str() + ".jpg")
            );
        }
        std::cout << "writing textured geometry..." << std::endl;
		const boost::filesystem::path outfile_geometry = outdir / (CONFIG.output + "_geometry");
        IFS::exportOBJ(exportIFS, outfile_geometry.string(), "# OrthoGen textured geometry\n");
    }

    // write spheres
    if (CONFIG.exportSphere)
    {
		const boost::filesystem::path panopath(CONFIG.panopath);

        for (auto &scan : STATE.scans) {
			const boost::filesystem::path outfile_sphere = outdir / (CONFIG.output + "_" + scan.basename);
			std::cout << "- exporting panorama sphere OBJ for " << scan.basename << std::endl;
            myIFS sphere = scan.exportTexturedSphere(CONFIG.sphereRadius, 100);
            sphere.materials.push_back(IFS::Material("sphere", (outdir.lexically_relative(panopath) / scan.panofile).string() ));
            sphere.facematerial.push_back(sphere.materials.size() - 1);
            IFS::exportOBJ(sphere, outfile_sphere.string(), "# OrthoGen panoramic sphere\n");
        }
    }

    std::cout << "### orthogen finished ###" << std::endl;
    
  } catch (std::exception &e) {
    std::cerr << "error: " << e.what() << "\n";
    return -1;
  }

}