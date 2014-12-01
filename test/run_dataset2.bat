@if not exist dataset2 mkdir dataset2
@cd dataset2
REM CONFIGURATION
set DATAPATH=I:\Projects\2014-04-DuraArk\DataSets\D5_2_TestDataSet02
set PANO=%DATAPATH%\LysLab_ColourOverlay_Scan03.ppm
set GEOMETRY=%DATAPATH%\LysLab_RoomGeo_Scan03.obj
set ORIENTATION=0.577755 0.000204727 -0.00437552 -0.816199
set TRANSLATION=0 0 -65.6473
set RESOLUTION=1
set ELMIN=-1.04627
set ELMAX=1.56941
set SCALE=m

..\..\MSVC2013\Release\orthogen.exe --im=%PANO% --ig=%GEOMETRY% --trans %TRANSLATION% --rot %ORIENTATION% --res %RESOLUTION% --elmin=%ELMIN% --elmax=%ELMAX% --exsphere 1 --exgeom 1 --scale %SCALE%


pause
