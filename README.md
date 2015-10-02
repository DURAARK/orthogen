# DuraArk Orthophoto Generation Module #

This tool extracts rectangular image patches from a given proxy geometry
(rectangular patches that correspond to walls) and panoramic photographs of a scene. 
It automatically combines the data from several scans / panoramic images.

See also the accompanying paper "Automatic Texture and Orthophoto Generation from registered panoramic views" (note that this paper was written for version 0.7.0)
http://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XL-5-W4/131/2015/isprsarchives-XL-5-W4-131-2015.html

Changes since version 0.7.0 include that the system now combines the 
information from several scans / panoramic images. It parses the room
network given from the IFC reconstruction and fits an oriented bounding 
box for each room in order to automatically assign scanner positions to rooms.

Furthermore, the project also creates an executable named "panoalign"
that automatically aligns a manually taken image to the registered image
from the faro scanner. The tool expects all panoramic images to be in 
an folder, where each image is named according to the scan id in the
e57 metadata: for each scan id "scan_id", the system tries to find an image
named "scan_id"_aligned.jpg that contains the aligned manual 
acquired panoramic image of the scan.
If the image is not found, the system tries to load the images
 "scan_id"_Manual.jpg and "scan_id"_Faro.jpg and tries to automatically
align the manual (high resolution) image  to the faro image using the
panoalign tool. If "scan_id"_Manual.jpg is not found, the system uses
the "scan_id"_Faro.jpg for projection as a last resort. If absolutely
no panoramic image can be found, the scan is ignored.

# Build #

The tool is written in C++ with dependencies on the following libraries:

* boost [program options]  http://www.boost.org/
* eigen http://eigen.tuxfamily.org/
* RapidJSON https://github.com/miloyip/rapidjson

CMake is used as build system.

# Run #

The tool is a simple commandline application. It requires the following
input mandatory input.

* a JSON file that contains the e57 metadata information from pointcloud scans
* a JSON file that contains the wall/room meta information from the IFC reconstruction
* a path to the panoramic images

**Command line arguments**

```
OrthoGen orthographic image generator for DuraArk
developed by Fraunhofer Austria Research GmbH
commandline options:

  --help                     show this help message
  --align arg                align executable
  --e57metadata arg          e57 metadata json [.json]
  --walljson arg             input wall json [.json]
  --exgeom arg               export textured geometry as .obj [0]/1
  --exroombb arg             export room bounding boxes as .obj [0]/1
  --output arg               output filename [.jpg] will be appended
  --panopath arg             path to pano images
  --resolution arg           resolution [1mm/pixel]
  --scanoffset arg           translation offset
  --usefaroimage arg         use pano from faro scanner
  --verbose arg              print out verbose messages
  --scan arg                 if specified, only this scan will be considered
  --wall arg                 if specified, only this wall will be considered
  --bbfitnormalprecision arg normal encoding precision for oriented bounding
                             box fit for rooms [8]
```

**Example usage**

```orthogen.exe --e57metadata e57metadata.json --walljson wall.json --panopath .\pano --align .\align\panoalign.exe --exgeom 1 --exroombb 1```
