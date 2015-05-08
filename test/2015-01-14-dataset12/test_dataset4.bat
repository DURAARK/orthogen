
REM DATASET 4.1
SET DS1PATH=I:\Projects\2014-04-DuraArk\DataSets\D5.2_TestDataSet04\Nygade_Scan1001
SET DS1IM=%DS1PATH%\Nygade_Scan1001_Panorama_autoalign.jpg
SET DS1IM2=%DS1PATH%\ColourOverlay_Nygade1001.jpg
SET DS1MG=%DS1PATH%\FhA_Socket_Nygade_Scan1001.obj

REM DATASET 4.2
SET DS2PATH=I:\Projects\2014-04-DuraArk\DataSets\D5.2_TestDataSet04\Nygade_Scan1002
SET DS2IM=%DS2PATH%\Nygade_Scan1002_Panorama_02_autoalign.jpg
SET DS2RG=%DS2PATH%\FhA_Socket_Nygade_Scan1002.obj

REM DATASET 4.3
SET DS3PATH=I:\Projects\2014-04-DuraArk\DataSets\D5.2_TestDataSet04\Nygade_Scan1005-1006
SET DS3IM=%DS3PATH%\Nygade_Scan1005_Panorama_02_autoalign.jpg
SET DS3RG=%DS3PATH%\FhA_Socket_Nygade_Scan1005-1006.obj

SET DS4IM=%DS3PATH%\Nygade_Scan1006_Panorama_02_autoalign.jpg
SET DS4RG=%DS3PATH%\FhA_Socket_Nygade_Scan1005-1006.obj

SET DS4prtIM=%DS3PATH%\Nygade_Scan1006_Panorama_02_autoalign.jpg
SET DS4prtRG=%DS3PATH%\FhA_Socket_Nygade_Scan1005-1006_part.obj


if not exist ds4_1001 mkdir ds4_1001
cd ds4_1001
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS1IM% --ig=%DS1MG% --rot 0.9592315236 -0.00766527459 -0.007286718304 0.2824234966 --trans 0 0 141.6600828 --res 1 --elevation -1.5707963 1.5707963 --scale m --exgeom  1 --exsphere 1 --exquad 1 > ds4_1001.txt
cd..

if not exist ds4_1002 mkdir ds4_1002
cd ds4_1002
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS2IM% --ig=%DS2RG% --rot 0.9170060213 -0.001173199767 0.0009580284973 -0.3988704837 --trans 0 0 138.3691565 --res 1 --elevation -1.5707963 1.5707963 --scale m --exgeom  1 --exsphere 1 --exquad 1 > ds4_1002.txt
cd..

if not exist ds4_1005 mkdir ds4_1005
cd ds4_1005
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS3IM% --ig=%DS3RG% --rot 0.35786571807044137 -0.0055577056971046349 0.0016985300220166769 -0.93375497574742605 --trans 0 0 136.76625579899999 --res 1 --elevation -1.5707963 1.5707963 --scale m --exgeom  1 --exsphere 1 --exquad 1 > ds4_1005.txt
cd..

if not exist ds4_1006 mkdir ds4_1006
cd ds4_1006
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS4IM% --ig=%DS4RG% --rot 0.21550253133271829 -0.0069849928162943982 -0.0031283649310417131 -0.97647328801017652 --trans 4.1043753438084121 -7.3179313861409874 136.77736476206837 --res 1 --elevation -1.5707963 1.5707963 --scale m --exgeom  1 --exsphere 1 --exquad 1 --dcluster 0.2 > ds4_1006.txt
cd..

if not exist ds4_1006_part mkdir ds4_1006_part
cd ds4_1006_part
..\..\..\MSVC2013\Release\orthogen.exe --im=%DS4prtIM% --ig=%DS4prtRG% --rot 0.21550253133271829 -0.0069849928162943982 -0.0031283649310417131 -0.97647328801017652 --trans 4.1043753438084121 -7.3179313861409874 136.77736476206837 --res 1 --elevation -1.5707963 1.5707963 --scale m --exgeom  1 --exsphere 1 --exquad 1 > ds4_1006_part.txt
cd..


echo.&pause&goto:eof
