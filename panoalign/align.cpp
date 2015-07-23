//
// panoramic image align tool
// ulrich.krispel@fraunhofer.at
// 

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
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
    Image imsrcResized;
    imsrcResized.initialize(C2, R2, imsrc.bpp());
    std::cout << "resizing source image.." << std::endl;
    resizeBlit(imsrc, imsrc.whole(), imsrcResized, Image::AABB2D(0, padhigh, C2, R1target));
    saveJPEG("srcresized.jpg", imsrcResized);

    auto NormalizeImage = [](const Image &img) -> ImageD
    {
        ImageD result = convertToGrayScale<Image, ImageD>(img);
        const double mean = result.mean<double>();
        const double std = result.std<double>(mean);
        {
            auto normalizeCB = [&result, &mean, &std](double pval)
            {
                return (pval - mean) / std;
            };
            result.transformPixelCB(normalizeCB, result.whole());
        }
        return result;
    };
    std::cout << "normalizing source image.." << std::endl;
    const ImageD src_n = NormalizeImage(imsrcResized);
    std::cout << "normalizing destination image.." << std::endl;
    const ImageD dst_n = NormalizeImage(imdst);

// perform SAD for horizontally shifted image, 1 pixel steps
    assert(src_n.width() == dst_n.width());
    std::vector<double> SAD;
    SAD.resize(src_n.width(), 0.0);

    std::cout << "searching for SAD optimum.." << std::endl;
    #pragma omp parallel for
    for (int shift = 0; shift < dst_n.width(); ++shift)
    {
        auto SAD_CB = [&SAD, &src_n, &dst_n, shift](int x, int y)
        {
            SAD[shift] += std::abs(src_n(x, y) - dst_n((x + shift) % dst_n.width(), y) );
        };
        imsrc.applyPixelPosCBS(SAD_CB, imsrc.whole());
    }
// create output image using best guess
    const std::vector<double>::iterator it = std::min_element(SAD.begin(), SAD.end());
    const int shift = std::distance(SAD.begin(), it);
    std::cout << "found optimum at shift " << shift << std::endl;
    std::cout << "creating and writing output image..." << shift << std::endl;

    Image FinalOutput;
    FinalOutput.initialize(imdst.width(), imdst.height(), imdst.bpp());
    auto ShiftCB = [&imdst, &FinalOutput,shift](int x, int y)
    {
        int x_ = (x + shift) % imdst.width();
        for (int ch = 0, che = imdst.channels(); ch < che; ++ch)
            FinalOutput(x, y, ch) = imdst(x_, y, ch);
    };
    FinalOutput.applyPixelPosCB(ShiftCB, FinalOutput.whole());
    saveJPEG("dst_aligned.jpg", FinalOutput);

// print positions of 5 largest values

}
