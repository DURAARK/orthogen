#ifndef _PNM_HPP_
#define _PNM_HPP_

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "image.h"

namespace PNM
{
    // namespace functions
    
    // utility function: parse line from input
    std::string readNextLine(std::istream &in);

    // background is RGB 24-bit color which will be set transparent
    
    // load image from input stream
    Image loadPNM(std::istream &in, const int background=-1);

    // load image from file
    Image loadPNM(const std::string &filename, const int background=-1);

    // write image to output stream
    bool writePNM(const Image &img, std::ostream &os);

    // write image to file
    bool writePNM(const Image &img, const std::string &fname);

} // namespace GLFONT

#endif
