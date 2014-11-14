if not exist test1 mkdir test1
cd test1
..\..\MSVC2013\Debug\orthogen.exe --im=..\Scan00_FaroPic.ppm --ig=..\RansacRegisteredPlanes.obj --rot 0.003361 0.002216 0.99964 0.026682 --res 1.0 --elmin=-1.046 --elmax=1.5694 > out.txt
pause
