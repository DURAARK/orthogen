REM PARAMS run.bat <output dir:%0> <data path:%1> <pano:%2> <geometry:%3> <orientation:%4> <translation:%5> <resolution:%6> <elmin:%7> <elmax:%8> <scale:%9>
if not exist %0 mkdir %0
cd %0
..\..\MSVC2013\Release\orthogen.exe --im=%1\%2 --ig=%1\%3 --rot %4 --trans %5 --res %6 --elmin %7 --elmax %8 --scale %9 --exgeom 1
