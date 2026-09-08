@echo off
setlocal
set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "REPOSITORY_ROOT=%%~fI"
set "BUILD_DIRECTORY=%REPOSITORY_ROOT%\build-viewer"

if defined VOLUME_SURFACE_VCVARS64 (
    if exist "%VOLUME_SURFACE_VCVARS64%" set "VCVARS64=%VOLUME_SURFACE_VCVARS64%"
)

if not defined VCVARS64 (
    for %%I in (
        "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
        "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
    ) do (
        if not defined VCVARS64 if exist "%%~fI" set "VCVARS64=%%~fI"
    )
)

if not defined VCVARS64 (
    echo Unable to find a Visual Studio 2022 x64 C++ toolchain.
    echo Set VOLUME_SURFACE_VCVARS64 to vcvars64.bat and run this script again.
    exit /b 1
)

if not exist "%REPOSITORY_ROOT%\.deps\filament-main\CMakeLists.txt" (
    echo Filament source is missing.
    echo Run: powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%REPOSITORY_ROOT%\scripts\prepare_filament.ps1"
    exit /b 1
)

for %%I in ("%VCVARS64%") do set "VCVARS_DIRECTORY=%%~dpI"
for %%I in ("%VCVARS_DIRECTORY%..\..\..") do set "VISUAL_STUDIO_ROOT=%%~fI"
set "VISUAL_STUDIO_CMAKE=%VISUAL_STUDIO_ROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
set "VISUAL_STUDIO_NINJA=%VISUAL_STUDIO_ROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja"
if exist "%VISUAL_STUDIO_CMAKE%\cmake.exe" set "PATH=%VISUAL_STUDIO_CMAKE%;%PATH%"
if exist "%VISUAL_STUDIO_NINJA%\ninja.exe" set "PATH=%VISUAL_STUDIO_NINJA%;%PATH%"

where cmake >nul 2>nul
if errorlevel 1 (
    echo CMake was not found on PATH.
    exit /b 1
)

where ninja >nul 2>nul
if errorlevel 1 (
    echo Ninja was not found on PATH.
    exit /b 1
)

call "%VCVARS64%" >nul
if errorlevel 1 exit /b %errorlevel%

echo Configuring Viewer in "%BUILD_DIRECTORY%"...
cmake -S "%REPOSITORY_ROOT%" -B "%BUILD_DIRECTORY%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DVOLUME_SURFACE_BUILD_TESTS=OFF -DVOLUME_SURFACE_BUILD_VIEWER=ON
if errorlevel 1 exit /b %errorlevel%

echo Building volume_surface_viewer...
cmake --build "%BUILD_DIRECTORY%" --target volume_surface_viewer
if errorlevel 1 exit /b %errorlevel%

echo Viewer is ready: "%BUILD_DIRECTORY%\volume_surface_viewer.exe"
