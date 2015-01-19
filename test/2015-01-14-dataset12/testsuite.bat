
REM DATASET 1
SET DS1PATH=I:\Projects\2014-04-DuraArk\DataSets\D5.2_TestDataSet01
SET DS1IM=%DS1PATH%\ColourOverlayExports\Scan00_AutoAlign.jpg
SET DS1MG=%DS1PATH%\OBJ\ModelledSpace.obj
SET DS1RG=%DS1PATH%\OBJ\RansacRegisteredPlanes.obj

REM DATASET 2
SET DS2PATH=I:\Projects\2014-04-DuraArk\DataSets\D5.2_TestDataSet02
SET DS2IM=%DS2PATH%\lyslab_panorama.jpg
SET DS2IM2=%DS2PATH%\LysLab_ColourOverlay_Scan03.jpg
SET DS2RG=%DS2PATH%\LysLab_RoomGeo_Scan03.obj

REM DATASET 3
SET DS3PATH=I:\Projects\2014-04-DuraArk\DataSets\D5.2_TestDataSet03
SET DS3IM=%DS3PATH%\ColourOverlay_Nygade037.jpg
SET DS3RG=%DS3PATH%\FhA_Socket_NygadeScan.obj


if not exist ds1_modelled mkdir ds1_modelled
cd ds1_modelled
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS1IM% --ig=%DS1MG% --rot 0.0266818 0.00336098 0.00221603 0.999636 --trans 0 0 0 --res 1 --elevation -1.570796327 1.570796327 --scale mm --exgeom  1 --exsphere 1
cd..

if not exist ds1_ransac mkdir ds1_ransac
cd ds1_ransac
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS1IM% --ig=%DS1RG% --rot 0.0266818 0.00336098 0.00221603 0.999636 --trans 0 0 0 --res 1 --elevation -1.570796327 1.570796327 --scale mm --exgeom  1 --exsphere 1
cd..

if not exist ds2_ransac mkdir ds2_ransac
cd ds2_ransac
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS2IM% --ig=%DS2RG% --rot 0.5777548965 0.0002047265214 -0.004375523157 -0.8161985619 --trans 0 0 -65.6473 --res 1 --elevation -1.570796327 1.570796327 --scale m --exgeom  1 --exsphere 1
cd..

if not exist ds2_ransac_faro mkdir ds2_ransac_faro
cd ds2_ransac_faro
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS2IM2% --ig=%DS2RG% --rot 0.5777548965 0.0002047265214 -0.004375523157 -0.8161985619 --trans 0 0 -65.6473 --res 1 --elevation -1.046266501 1.569409603 --scale m --exgeom  1 --exsphere 1
cd..

if not exist ds3_ransac mkdir ds3_ransac
cd ds3_ransac
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS3IM% --ig=%DS3RG% --rot 0.3473116843 -0.005407991293 -0.003936551737 0.9377258934 --trans 0 0 0.85 --res 1 --elevation -1.046266501 1.570022792 --scale m --exgeom  1 --exsphere 1
cd..


pause