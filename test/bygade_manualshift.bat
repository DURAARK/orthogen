REM manual shifts for images that could not be aligned automatically
REM in the Byg72 dataset

set PANO=I:\Projects\2014-04-DuraArk\DataSets\D5.4_Byg72\pano
set ELEVATION=-1.0459598204934397 1.5694096034770006

set bindir=I:\Projects\2014-04-DuraArk\dev\OrthoGen\MSVC2013_W64\Release
set align=%bindir%\panoalign.exe

REM ====================== JUST PERFORM SHIFT

rem set SHIFT=1800
rem set IMG=Byg72_023
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=3100
rem set IMG=Byg72_004008
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=1450
rem set IMG=Byg72_005009
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=2950
rem set IMG=Byg72_006012
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=4650
rem set IMG=Byg72_008015
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=6250
rem set IMG=Byg72_009016
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=13050
rem set IMG=Byg72_009017
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=6450
rem set IMG=Byg72_lotz027
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

rem set SHIFT=9750
rem set IMG=Byg72_lotz028
rem %align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%

set SHIFT=250
set IMG=Byg72_024
%align% --imsrc %PANO%\%IMG%_Faro.jpg --imdst %PANO%\%IMG%_Manual.jpg --outname %PANO%\%IMG%_aligned.jpg --selrange %ELEVATION% --shift %SHIFT%


pause
