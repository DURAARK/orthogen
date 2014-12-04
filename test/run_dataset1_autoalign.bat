@if not exist dataset1 mkdir dataset1
@cd dataset1
REM CONFIGURATION
set DATAPATH=I:\Projects\2014-04-DuraArk\DataSets\D5_2_TestDataSet01
set PANO=%DATAPATH%\ColourOverlayExports\Scan00_AutoAlign.ppm
set GEOMETRY=%DATAPATH%\OBJ\RansacRegisteredPlanes.obj
set ORIENTATION=0.0266818 0.00336098 0.00221603 0.999636
set TRANSLATION=0 0 0
set RESOLUTION=1
set ELMIN=-1.570796327
set ELMAX=1.570796327
set SCALE=mm

..\..\MSVC2013\Release\orthogen.exe --im=%PANO% --ig=%GEOMETRY% --trans %TRANSLATION% --rot %ORIENTATION% --res %RESOLUTION% --elmin=%ELMIN% --elmax=%ELMAX% --exsphere 1 --exgeom 1 --scale %SCALE%

pause
