# DuraArk Orthophoto Generation Module #

This tool extracts rectangular image patches from a given proxy geometry
(rectangular patches that correspond to walls) and panoramic photographs of a scene. 
It automatically combines the data from several scans / panoramic images.

See also the accompanying paper "Automatic Texture and Orthophoto Generation from registered panoramic views" (note that this paper was written for version 0.7.0)
http://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XL-5-W4/131/2015/isprsarchives-XL-5-W4-131-2015.html

Updates for version 0.9.0 include that the system now combines the 
information from several scans / panoramic images. It parses the room
network given from the IFC reconstruction and fits an oriented bounding 
box for each room in order to automatically assign scanner positions to rooms.

Updates for version 0.10.1 include a better fusion for datasets consisting
of several scans: the color projection for each patch now detects if a
scan is occluded by the input geometry (e.g. a wall), so that color
gets only projected on patches that are visible from that scan.
This version also adds a new input format, as the e57 metadata json format
is not as easily accessible. A tool that extracts the pose information
from an .e57 file is included in the "src/e57" subfolder. The panoramic
images are supposed to be named after the corresponding scan "name" tag in the config (+.jpg).

In Version 0.7.0 and 0.9.0, the software also may 
align a manually taken image to the registered image from the faro scanner. 
The tool expects all panoramic images to be in 
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

* boost [program options, file system]  http://www.boost.org/
* eigen http://eigen.tuxfamily.org/
* RapidJSON https://github.com/miloyip/rapidjson
(* libe57 for standalone pose extraction tool)

CMake is used as build system.

# Run #

The tool is a simple commandline application. It requires the following
input mandatory input.

* a JSON file that contains the e57 metadata information from pointcloud scans
* a JSON file that contains the wall/room meta information from the IFC reconstruction
* a path to the panoramic images

## Config ##

As input configuration, the pose information for each scan is needed. The
format is similar to the pose information found in .e57 point cloud files,
each pose consists of a 3D position, a rotation (quaternion), and the
elevation and azimuth bounds of the scan.

Example config:
```
{  
   "id":"{9053ACBF-14F1-4A3D-9B0A-1A665CF2E978}",
   "file":"scan.e57",
   "scans":[  
      {  
         "name":"Scan_014",
         "pos":{  
            "x":-9.262907604,
            "y":2.93273054,
            "z":-0.43658123
         },
         "rot":{  
            "x":-0.004104704979260176,
            "y":-0.0008693324679289275,
            "z":0.5661924748596193,
            "w":0.8242623836318342
         },
         "bounds":{  
            "elevation_minimum":-1.090429945492252,
            "elevation_maximum":1.5700227915171317,
            "azimuth_start":6.267296923414599,
            "azimuth_end":-0.0028747671873452405
         }
      },
      ...
      ...
   ]
}
```

## Command line arguments ##

```
OrthoGen orthographic image generator for DuraArk
developed by Fraunhofer Austria Research GmbH
commandline options:

  --help                show this help message
  --e57metadata arg     e57 metadata json [.json]
  --configjson arg      json file with pose information
  --geometry arg        input geometry [.obj]
  --align arg           align executable
  --wnddist arg         distance clustering window size [0.1]
  --wndnorm arg         normal clustering window size [0.3]
  --exgeom arg          export textured geometry as .obj [0]/1
  --exortho arg         export orthophotos as .jpg 0/[1]
  --exsphere arg        export textured panoramic sphere(s) as .obj [0]/1
  --exquad arg          export textured quads as .obj [0]/1
  --excluster arg       export triangle clusters as .obj [0]/1
  --output arg          output filename [.jpg] will be appended
  --panopath arg        path to pano images
  --resolution arg      resolution [mm/pixel]
  --scan arg            if specified, only this scan will be considered
  --scanoffset arg      translation offset
  --verbose arg         print out verbose messages
  --sphereradius arg    exported panoramic sphere radius [1.0]
```

use either --e57metadata for the DURAARK e57 metadata json format, or
--configjson to use the JSON format as described in Config above.

**Example usage**

```orthogen --configjson scan.json --geometry proxygeometry.obj --resolution 1 --wnddist 0.1 --exquad 1 --exortho 1 --exsphere 1```
