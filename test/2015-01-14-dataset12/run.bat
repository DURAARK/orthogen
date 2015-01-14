REM PARAMS run.bat <output dir:%1> <data path:%2> <pano:%3> <geometry:%4> <orientation:%5> <translation:%6> <resolution:%7> <elmin:%8> <elmax:%9> <scale:%10>
if not exist %1 mkdir %1
cd %1
..\..\..\MSVC2013\Release\orthogen.exe --im=%2\%3 --ig=%2\%4 --rot %5 --trans %6 --res %7 --elevation %8 --scale %9 --exgeom 1
cd..