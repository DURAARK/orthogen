# DuraArk Orthophoto Generation Module #

This tool extracts rectangular image patches from a given proxy geometry
and a panoramic photograph of a scene. The patches are automatically
identified from elements (triangles, quads) of the input geometry.

See also the accompanying paper "Automatic Texture and Orthophoto Generation from registered panoramic views":
http://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XL-5-W4/131/2015/isprsarchives-XL-5-W4-131-2015.html

# Build #

The tool is written in C++ with dependencies on the following libraries:

* boost [program options]  http://www.boost.org/
* eigen http://eigen.tuxfamily.org/

CMake is used as build system.

# Run #

The tool is a simple commandline application. It accepts the following
input:

* the input geometry as an wavefront .OBJ file
* the panoramic image as a .JPG, with elevation and azimuth range
* pose information of the panoramic image with respect to the .OBJ
coordinate system. This information is typically acquired from a 3D scan.
The pose information is compatible with the pose information stored in E57
format (translation x,y,z and quaternion rotation w,x,y,z)
* for the quad clustering, two parameters have to be supplied:
** "normal direction clustering window size" influences the difference between normal 
vector orientations that are clustered together, so a value means more 
dissimilar normals will be clustered together (a value >2 makes no sense)
** "distance clustering window size" specifies which elements of similar normal direction
are grouped into one quad, meaning that a higher value will group parallel elements
within this distance size into one plane (distance measure in m)

Command line arguments:

`OrthoGen orthographic image generator for DuraArk
developed by Fraunhofer Austria Research GmbH
commandline options:
  --help                show this help message
  --im arg              input panoramic image [.jpg]
  --ig arg              input geometry [.OBJ]
  --res arg             resolution [mm/pixel], default 1mm/pixel
  --trans arg           transformation [x,y,z]
  --rot arg             rotation quaternion [w,x,y,z]
  --elevation arg       elevation angle bounds [min..max], default
                        [-PI/2..PI/2]
  --azimuth arg         azimuth angle bounds [min..max], default [0..2*PI]
  --exgeom arg          export (textured) geometry [OBJ] 0/1, default 0 (false)
  --exquad arg          export extracted quads as (textured) geometry [OBJ]
                        0/1, default 0 (false)
  --exsphere arg        export panoramic sphere [OBJ] 0/1, default 0 (false)
  --scale arg           scale of input coordinates (mm/cm/m), default m
  --ncluster arg        geometry element normal direction clustering window
                        size, default 0.3
  --dcluster arg        geometry element planar distance clustering window size
                        in m, default 0.1`

