
#include "pnm.h"

namespace PNM
{

   std::string readNextLine(std::istream &in)
    {
    	std::string line;
    	std::string firstpart;
    	do
    	{
    	  getline(in, line);
    	  std::stringstream trimmer;
          firstpart.clear();
    	  trimmer << line;
    	  trimmer >> firstpart;
    	} while(!in.eof() && !firstpart.empty() && firstpart.at(0)=='#');
    	return line;
    }

    // this assumes that the stream is valid.
    Image loadPNM(std::istream &in, const int background)
    {
        Image I;
        std::string line;
        bool binary;

		int bpp, width, height;

        // read magic number
        line = readNextLine(in);
        if(line.at(0)!='P')
        {
            return Image();
        }
        switch(line.at(1))
        {
            case '1': bpp = 1;  binary = false; break;
			case '2': bpp = 8;  binary = false; break;
			case '3': bpp = 24; binary = false; break;
			case '4': bpp = 1;  binary = true; break;
			case '5': bpp = 8;  binary = true; break;
			case '6': bpp = 24; binary = true; break;
            default : std::cout << "unknown pbm type ";
             //<< line.at(1) << std::endl;
        }

        // read width and height
        {
            line = readNextLine(in);
            std::stringstream sstr(line);
            sstr >> width;
            sstr >> height;
        }

        if (I.bpp() > 1)
        {
            int maxValue = 0;
            line = readNextLine(in);
            std::stringstream sstr(line);
            sstr >> maxValue;
        }

        // read pixels
		I.initialize(width, height, bpp);
        const unsigned int bufsize=I.buffersize();

        if (binary)
        {
           in.read( (char *)I.data(), bufsize);
        }
        else
        {
            // special handling for 1-bit images
            if (I.bpp() > 1)
            {
              int value;
              for (unsigned int i=0; i<bufsize; ++i)
              {
                in >> value;
                if (value > 255) value=255;
				I(i) = value;
              }
            }
            else
            {
               int value;

               for (unsigned int i=0,N=I.width()*I.height(); i<N; ++i)
               {
                  in >> value;
                  if (value) I(i >> 3) |= 0x80 >> (i % 8);
               }
            }
        }

//        // alpha channel creation is only supported for 24bit pnms
//        if (background != -1 && I.bpp() == 24)
//        {
//            unsigned int bufsize=I.width()*I.height()*32/8;
//            std::vector<unsigned char> pdata;
//            pdata.resize(bufsize);
//            int si=0;
//            int di=0;

//            for(int y=0;y<I.height();++y)
//            {
//              for(int x=0;x<I.width();++x)
//              {
//                // copy rgb
//                unsigned char r=I(si++);
//                unsigned char g=I(si++);
//                unsigned char b=I(si++);
//                pdata[di++]=r;
//                pdata[di++]=g;
//                pdata[di++]=b;
//                int color=r+(g<<8)+(b<<16);
//                pdata[di++]= (color == background) ? 0x00 : 0xFF;
//              }
//            }

//			I.initialize(I.width(), I.height(), 32);
//			I.unsafeData() = pdata;
//        }

        return I;
    }

    Image loadPNM(const std::string &filename, const int background)
    {
        std::ifstream in(filename, std::ios::binary);
        return loadPNM(in,background);
    }
    
}
