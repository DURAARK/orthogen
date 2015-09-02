//
// panoramic image align tool
// ulrich.krispel@fraunhofer.at
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <iomanip>
#include <boost/program_options.hpp>

// images/jpg
#include "../src/image.h"
#include "../src/meanshift.h"

// use eigen matrices
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace po = boost::program_options;

#define PI 3.14159265358979323846
#define PI2 (PI / 2.0)

int main(int ac, char *av[]) {
  Image imsrc, imdst;
  double selmin = -PI2, selmax = PI2, delmin = -PI2, delmax = PI2;
  int optimum = 0;
  int y_offset = 0;
  std::string srcname = "src.jpg";
  std::string dstname = "dst.jpg";

  po::options_description desc("commandline options");

  // read input panoramas and opening angle, convert to grayscale
  desc.add_options()("help", "show this help message")
      ("imsrc,S", po::value<std::string>(), "source panoramic image [.JPG]")
      ("imdst,D", po::value<std::string>(), "destination panoramic image [.JPG]")
      ("writesad", po::value<int>(),"export column SAD as csv")
      ("readsad", po::value<int>(),"import column SAD as csv, do not calculate SAD optimum")
      ("shift", po::value<int>(), "just perform shift")
      ("yoff", po::value<int>(),"Y-Offset [0]")
      ("writenormalized", po::value<int>(), "also write normalized images")
      ("selrange", po::value<std::vector<double>>()->multitoken(),"source elevation angle bounds [min..max], default [-PI/2..PI/2]")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc,
                                   po::command_line_style::unix_style ^
                                       po::command_line_style::allow_short),
            vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  if (vm.count("selrange")) {
    std::vector<double> el = vm["selrange"].as<std::vector<double>>();
    selmin = el[0];
    selmax = el[1];
  }
  if (vm.count("delrange")) {
    std::vector<double> el = vm["delrange"].as<std::vector<double>>();
    delmin = el[0];
    delmax = el[1];
  }
  if (vm.count("imsrc")) {
    srcname = vm["imsrc"].as<std::string>();
  }
  if (vm.count("imdst")) {
    dstname = vm["imdst"].as<std::string>();
  }
  if (vm.count("optimum")) {
    optimum = vm["optimum"].as<int>();
  }
  if (vm.count("yoff")) {
    y_offset = vm["yoff"].as<int>();
  }
  std::cout << "y_offset: " << y_offset << std::endl;
  // load images
  std::cout << "loading " << dstname << std::endl;
  loadJPEG(dstname.c_str(), imdst);
  if (!imdst.isValid()) {
    return -1;
  }

  if (vm.count("shift")) {
    const int shift = vm["shift"].as<int>();
    Image FinalOutput;
    FinalOutput.initialize(imdst.width(), imdst.height(), imdst.bpp());
    auto ShiftCB = [&imdst, &FinalOutput, shift, y_offset](int x, int y) {
      int x_ = (x + shift) % imdst.width();
      int y_ = (y + y_offset);
      if (y_ < 0) {
        y_ += imdst.height();
      } else {
        y_ %= imdst.height();
      }
      for (int ch = 0, che = imdst.channels(); ch < che; ++ch)
        FinalOutput(x, y, ch) = imdst(x_, y_, ch);
    };
    FinalOutput.applyPixelPosCB(ShiftCB, FinalOutput.whole());
    saveJPEG("dst_aligned.jpg", FinalOutput);
    return 0;
  }

  std::cout << "loading " << srcname << std::endl;
  loadJPEG(srcname.c_str(), imsrc);
  if (!imsrc.isValid()) {
    return -1;
  }

  std::vector<double> SAD;
  if (!vm.count("readsad")) {
    // resample and normalize
    int R1 = imsrc.height(), C1 = imsrc.width();
    int R2 = imdst.height(), C2 = imdst.width();

    // calculate height of R1
    int R1target = (R1 * C2) / C1;
    int padlow = (int)round((R2 / 2.0) * (PI2 + selmin) / PI2);
    int padhigh = (int)round((R2 / 2.0) * (PI2 - selmax) / PI2);

    std::cout << "source elevation: " << selmin << " <-> " << selmax
              << std::endl;
    std::cout << "alignment size: " << C2 << "x" << R2 << " pixels, "
              << " padding: " << padlow << " low " << padhigh << " high"
              << std::endl;
    Image imsrcResized;
    imsrcResized.initialize(C2, R2, imsrc.bpp());
    std::cout << "resizing source image.." << std::endl;
    resizeBlit(imsrc, imsrc.whole(), imsrcResized,
               Image::AABB2D(0, padhigh, C2, R1target));
    saveJPEG("srcresized.jpg", imsrcResized);

    auto NormalizeImage = [](const Image &img) -> ImageD {
      ImageD result = convertToGrayScale<Image, ImageD>(img);
      const double mean = result.mean<double>();
      const double std = result.std<double>(mean);
      {
        auto normalizeCB = [&result, &mean, &std](double pval) {
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

    if (vm.count("writenormalized")) {
      auto writeNormalized = [](const ImageD &img, const std::string &fn) {
        Image out;
        out.resize(img.width(), img.height(), 3);
        const double dmin = img.min(), dmax = img.max();
        std::cout << "min:" << dmin << " max: " << dmax << std::endl;
        auto CB = [&img, &out, dmin, dmax](int x, int y) {
          out(x, y, 0) = (img(x, y, 0) - dmin) * 255.0 / (dmax - dmin);
          out(x, y, 1) = (img(x, y, 0) - dmin) * 255.0 / (dmax - dmin);
          out(x, y, 2) = (img(x, y, 0) - dmin) * 255.0 / (dmax - dmin);
        };
        out.applyPixelPosCB(CB, out.whole());
        saveJPEG(fn.c_str(), out);
      };
      writeNormalized(src_n, "src_normalized.jpg");
      writeNormalized(dst_n, "dst_normalized.jpg");
    }
    // perform SAD for horizontally shifted image, 1 pixel steps

    assert(src_n.width() == dst_n.width());
    SAD.resize(src_n.width(), 0.0);

    std::cout << "searching for SAD optimum.." << std::endl;
#pragma omp parallel for
    for (int shift = 0; shift < dst_n.width(); ++shift) {
      auto SAD_CB = [&SAD, &src_n, &dst_n, shift](int x, int y) {
        SAD[shift] +=
            std::abs(src_n(x, y) - dst_n((x + shift) % dst_n.width(), y));
      };
      imsrc.applyPixelPosCBS(SAD_CB, imsrc.whole());
    }
  } else {
    std::ifstream in("sad.csv");

    while (!in.eof()) {
      std::string line;
      std::getline(in, line);
      if (line.length() > 0) {
        std::istringstream ss(line);
        int id;
        char delim;
        double value;
        ss >> id >> delim >> value;
        assert(SAD.size() == id);
        SAD.push_back(value);
      }
    }
  }

  /*
  // create output image using best guess
  const std::vector<double>::iterator it = std::min_element(SAD.begin(),
  SAD.end());
  const int shift = std::distance(SAD.begin(), it);
  */

  if (vm.count("writesad")) {
    std::ofstream of("sad.csv");
    for (int i = 0, ie = SAD.size(); i < ie; ++i) {
      of << i << "; " << std::fixed << std::setprecision(12) << SAD[i]
         << std::endl;
    }
  }

  //// find local minima
  // const int windowsize = SAD.size() / 10;
  // for (int i = 0, ie = SAD.size(); i < ie; ++i)
  //{

  //}

  std::cout << "find SAD modes.." << std::endl;
  MEANSHIFT::Meanshift<Vec1d, 1> ms_sad;
  double sadmax = DBL_MIN, sadmin = DBL_MAX;
  for (auto const &s : SAD) {
    Vec1d d;
    d[0] = s;
    if (s > sadmax)
      sadmax = s;
    if (s < sadmin)
      sadmin = s;
    ms_sad.points.push_back(d);
  }
  ms_sad.calculate((sadmax - sadmin) / 10.0);
  int i = 0;
  for (auto cluster = ms_sad.cluster.begin(); i < 5; ++cluster, ++i) {
    std::cout << " optimum " << i << " : " << cluster->first << std::endl;
    double localmin = DBL_MAX;
    int localid = -1;
    for (auto const &p : cluster->second) {
      if (SAD[p] < localmin) {
        localmin = SAD[p];
        localid = p;
      }
    }
    std::cout << " minimum: " << localmin << " [" << localid << "]"
              << std::endl;
    std::cout << std::endl;
  }

  // put into map (sort)
  std::map<double, int> sapcost;
  for (int i = 0, ie = SAD.size(); i < ie; ++i) {
    sapcost[SAD[i]] = i;
  }

  auto it = sapcost.begin();
  for (int i = 0; i < optimum; ++i) {
    ++it;
  }
  const int shift = it->second;
  std::cout << "found optimum " << optimum << " at shift " << shift
            << std::endl;
  std::cout << "creating and writing output image..." << shift << std::endl;

  Image FinalOutput;
  FinalOutput.initialize(imdst.width(), imdst.height(), imdst.bpp());
  auto ShiftCB = [&imdst, &FinalOutput, shift](int x, int y) {
    int x_ = (x + shift) % imdst.width();
    for (int ch = 0, che = imdst.channels(); ch < che; ++ch)
      FinalOutput(x, y, ch) = imdst(x_, y, ch);
  };
  FinalOutput.applyPixelPosCB(ShiftCB, FinalOutput.whole());
  saveJPEG("dst_aligned.jpg", FinalOutput);

  it = sapcost.begin();
  for (int i = 0; i < 5; ++i) {
    std::cout << "optimum " << i << ": at " << it->second << std::endl;
    ++it;
  }
}
