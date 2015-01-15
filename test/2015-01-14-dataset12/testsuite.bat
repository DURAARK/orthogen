
REM DATASET 1
SET DS1PATH=I:\Projects\2014-04-DuraArk\DataSets\D5_2_TestDataSet01
SET DS1IM=%DS1PATH%\ColourOverlayExports\Scan00_AutoAlign.jpg
SET DS1MG=%DS1PATH%\OBJ\ModelledSpace.obj
SET DS1RG=%DS1PATH%\OBJ\RansacRegisteredPlanes.obj

REM DATASET 2
SET DS2PATH=I:\Projects\2014-04-DuraArk\DataSets\D5_2_TestDataSet02
SET DS2IM=%DS2PATH%\lyslab_panorama.jpg
SET DS2RG=%DS2PATH%\LysLab_RoomGeo_Scan03.obj


rem if not exist ds1_modelled mkdir ds1_modelled
rem cd ds1_modelled
rem ..\..\..\MSVC2013\Release\orthogen.exe --im=%DS1IM% --ig=%DS1MG% --rot 0.0266818 0.00336098 0.00221603 0.999636 --trans 0 0 0 --res 1 --elevation -1.570796327 1.570796327 --scale mm --exgeom  1
rem cd..

rem if not exist ds1_ransac mkdir ds1_ransac
rem cd ds1_ransac
rem ..\..\..\MSVC2013\Release\orthogen.exe --im=%DS1IM% --ig=%DS1RG% --rot 0.0266818 0.00336098 0.00221603 0.999636 --trans 0 0 0 --res 1 --elevation -1.570796327 1.570796327 --scale mm --exgeom  1
rem cd..

if not exist ds2_ransac mkdir ds2_ransac
cd ds2_ransac
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS2IM% --ig=%DS2RG% --rot 0.0266818 0.00336098 0.00221603 0.999636 --trans 0 0 0 --res 1 --elevation -1.570796327 1.570796327 --scale mm --exgeom  1
cd..

pause