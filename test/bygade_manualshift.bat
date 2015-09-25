REM manual shifts for images that could not be aligned automatically
REM in the Byg72 dataset

set PANO=I:\Projects\2014-04-DuraArk\DataSets\D5.4_Byg72\pano
set ELEVATION=-1.0459598204934397 1.5694096034770006

set bindir=I:\Projects\2014-04-DuraArk\dev\OrthoGen\MSVC2013_W64\Release
set align=%bindir%\panoalign.exe

REM JUST PERFORM SHIFT

rem set SHIFT=1800
rem set IMG=Byg72_023
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=3100
rem set IMG=Byg72_004008
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

set SHIFT=1450
set IMG=Byg72_005009
%align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

pause
