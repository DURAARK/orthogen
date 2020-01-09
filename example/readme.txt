This directory contains a synthetic example for orthogen
to create cube map faces from a spherical panorama - 
since orthogen can project spherical panoramas on any (flat) geometry,
it can also project a spherical panorama (assumed at the origin) onto a
cube with its center in the origin.

spherical panorama from Jöriseen was taken from https://commons.wikimedia.org/wiki/File:J%C3%B6riseen_Spherical_Panorama.jpg

files:

* panorama.jpg : the spherical panorama
* cubemap.obj : the cubemap geometry
* config.json : the config file (cubemap in origin center)

start with:

 orthogen.exe --configjson config.json --geometry cubemap.obj --exortho 1 --exgeom 1 --exsphere 1 --output cubemap

to create cube map textures, the textured cube itself and a textured sphere.

Note: in order to view the textured sphere geometry, you need to put the 
panoramic image (panorama.jpg) in the same directory as the generated sphere geometry (cubemap_panorama.obj), i.e. the orthogen output directory
