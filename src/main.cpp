
#include "image.h"
#include "pnm.h"
#include "ifs.h"
#include "vec3.h"

#include <vector>
#include <string>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int ac, char* av[])
{
    Image inpano;
    IFS::IFS< 

    std::cout << "OrthoGen orthographic image generator for DuraArk" << std::endl;

    try
    {
        po::options_description desc("commandline options");
        desc.add_options()
            ("help", "show this help message")
            ("im", po::value< std::string >(), "input panoramic image [.PNM]")
            ("ig", po::value< std::string >(), "input geometry [.OBJ]")
        ;

        po::variables_map vm;        
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if (vm.count("help")) 
        {
            std::cout << desc << "\n";
            return 0;
        }

        if (vm.count("im")) 
        {
            // load pnm image
            PNM::loadPNM(vm["im"].as<std::string>());
        }

        if (vm.count("ig"))
        {
            // load pnm image
            
        }


    }
    catch(std::exception& e) 
    {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    if (inpano.isValid())
    {
        return 0;
    }

    std::cout << "--help for options." << std::endl;

}