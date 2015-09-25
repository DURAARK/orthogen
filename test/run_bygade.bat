set DATA=I:\Projects\2014-04-DuraArk\DataSets\D5.4_Byg72
set ORTHOGEN=I:\Projects\2014-04-DuraArk\dev\OrthoGen\MSVC2013_W64\Release\orthogen.exe
set ALIGN=I:\Projects\2014-04-DuraArk\dev\OrthoGen\MSVC2013_W64\Release\panoalign.exe
cd Byg72

%ORTHOGEN% --e57metadata %DATA%\Byg72_e57metadata.json --walljson %DATA%\Byg72_wall.json --panopath %DATA%\pano --align %ALIGN% --exgeom 1 
rem --usefaroimage 1
pause
