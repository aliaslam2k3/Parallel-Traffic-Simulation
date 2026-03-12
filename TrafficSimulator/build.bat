@echo off
REM Build script for Windows (MinGW/g++)
REM Ensure g++ and SFML are in your PATH

set CXX=g++
set CXXFLAGS=-std=c++17 -Wall -O2
set LDFLAGS=-lsfml-graphics -lsfml-window -lsfml-system

echo Building Serial version...
%CXX% %CXXFLAGS% -o TrafficSimulator_serial main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp %LDFLAGS%
if %ERRORLEVEL% neq 0 goto :error

echo Building OpenMP version...
%CXX% %CXXFLAGS% -DUSE_OPENMP -fopenmp -o TrafficSimulator_openmp main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp %LDFLAGS% -fopenmp
if %ERRORLEVEL% neq 0 goto :error

echo.
echo Build successful!
echo   TrafficSimulator_serial.exe  - Serial version
echo   TrafficSimulator_openmp.exe  - OpenMP parallel version
goto :end

:error
echo Build failed!
exit /b 1

:end
