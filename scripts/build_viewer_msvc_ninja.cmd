@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b %errorlevel%
cmake -S . -B build-viewer -G Ninja -DCMAKE_BUILD_TYPE=Release -DVOLUME_SURFACE_BUILD_TESTS=OFF -DVOLUME_SURFACE_BUILD_VIEWER=ON
if errorlevel 1 exit /b %errorlevel%
cmake --build build-viewer --target volume_surface_viewer
