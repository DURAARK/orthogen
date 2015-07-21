//
// panoramic image align tool
// ulrich.krispel@fraunhofer.at
// 

#include <vector>
#include <string>
#include <boost/program_options.hpp>

// images/jpg
#include "../src/image.h"

// use eigen matrices
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace po = boost::program_options;

#define PI 3.14159265358979323846
#define PI2 PI/2.0

int main(int ac, char* av[])
{
    Image imsrc, imdst;
    double selmin=-PI2, selmax=PI2, delmin=-PI2, delmax=PI2;


    po::options_description desc("commandline options");

// read input panoramas and opening angle, convert to grayscale
    desc.add_options()
        ("help", "show this help message")
        ("imsrc,S", po::value< std::string >()->default_value("src.jpg"), "source panoramic image [.JPG]")
        ("imdst,D", po::value< std::string >()->default_value("dst.jpg"), "destination panoramic image [.JPG]")
        ("selrange", po::value< std::vector<double> >()->multitoken(), "source elevation angle bounds [min..max], default [-PI/2..PI/2]")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc, po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 0;
    }
    if (vm.count("selrange"))
    {
        std::vector<double> el = vm["elevation"].as<std::vector<double> >();
        selmin = el[0];
        selmax = el[1];
    }
    if (vm.count("delrange"))
    {
        std::vector<double> el = vm["elevation"].as<std::vector<double> >();
        delmin = el[0];
        delmax = el[1];
    }

    // load images
    std::cout << "loading " << vm["imsrc"].as<std::string>() << std::endl;
    loadJPEG(vm["imsrc"].as<std::string>().c_str(), imsrc);
    if (!imsrc.isValid()) {
        return -1;
    }
    std::cout << "loading " << vm["imdst"].as<std::string>() << std::endl;
    loadJPEG(vm["imdst"].as<std::string>().c_str(), imdst);
    if (!imdst.isValid()) {
        return -1;
    }

    // resample and normalize
    int R1 = imsrc.height(), C1 = imsrc.width();
    int R2 = imdst.height(), C2 = imdst.width();

    // calculate height of R1
    int R1target = (R1*C2) / C1;
    int padlow  = (int) round((R2 / 2.0) * (PI2 + selmin) / PI2);
    int padhigh = (int) round((R2 / 2.0) * (PI2 - selmax) / PI2);

    std::cout << "alignment size: " << C2 << "x" << R2 << " pixels, " 
          << " padding: " << padlow << " low " << padhigh << " high" << std::endl;

    ImageD S1 = convertToGrayScale<Image,ImageD>(imsrc);
    const double S1mean = S1.mean<double>();
    const double S1std = S1.std<double>(S1mean);
    {
        auto normalizeCB = [&S1, &S1mean, &S1std](double pval)
        {
            return (pval - S1mean) / S1std;
        };
        S1.applyPixelCB(normalizeCB);
        Image E = convertImage<ImageD, Image>(S1);
        saveJPEG("src_gray.jpg", E);
    }

// perform SAD for horizontally shifted image, 1 pixel steps

// create output image using largest value

// print positions of 5 largest values

}
