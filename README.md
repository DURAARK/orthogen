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


