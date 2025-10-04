@echo off
REM Build script for octree-lib on Windows

setlocal

set BUILD_TYPE=%1
if "%BUILD_TYPE%"=="" set BUILD_TYPE=Release

set BUILD_DIR=build

echo === Building octree-lib in %BUILD_TYPE% mode ===

REM Create and enter build directory
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd "%BUILD_DIR%"

REM Configure
echo Configuring...
cmake -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ..
if errorlevel 1 goto :error

REM Build
echo Building...
cmake --build . --config %BUILD_TYPE%
if errorlevel 1 goto :error

REM Run tests
echo.
echo === Running tests ===
ctest -C %BUILD_TYPE% --output-on-failure
if errorlevel 1 goto :error

echo.
echo === Build complete ===
echo Run examples:
echo   %BUILD_TYPE%\basic_usage.exe
echo   %BUILD_TYPE%\spatial_queries.exe
echo   %BUILD_TYPE%\custom_data.exe
echo.
echo Run benchmarks:
echo   %BUILD_TYPE%\octree_benchmarks.exe

goto :end

:error
echo Build failed!
exit /b 1

:end
endlocal
