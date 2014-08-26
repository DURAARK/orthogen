if not exist test2 mkdir test2
cd test2
..\..\msvc32\Release\orthogen.exe --im=..\Scan00_Hdr.ppm --ig=..\RansacRegisteredPlanes.obj --rot 0.003361 0.002216 0.99964 0.026682 --res 1.0 --elmin=-1.046 --elmax=1.5694
pause
