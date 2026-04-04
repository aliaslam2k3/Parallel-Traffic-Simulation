@echo off
REM Build script for Windows (MinGW/g++)
REM Ensure g++ and SFML are in your PATH

set CXX=g++
set CXXFLAGS=-std=c++17 -Wall -O2
set LDFLAGS=-lsfml-graphics -lsfml-window -lsfml-system
set SRC=main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp
set SRC_MPI=mpi_main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp

echo Building Serial version...
%CXX% %CXXFLAGS% -o TrafficSimulator_serial.exe %SRC% %LDFLAGS%
if %ERRORLEVEL% neq 0 goto :error

echo Building OpenMP version...
%CXX% %CXXFLAGS% -DUSE_OPENMP -fopenmp -o TrafficSimulator_openmp.exe %SRC% %LDFLAGS% -fopenmp
if %ERRORLEVEL% neq 0 goto :error

where mpic++ >nul 2>&1
if %ERRORLEVEL% equ 0 (
  echo Building MPI version...
  mpic++ %CXXFLAGS% -DUSE_MPI -o TrafficSimulator_mpi.exe %SRC_MPI% %LDFLAGS%
  if %ERRORLEVEL% neq 0 goto :error
) else (
  echo Skipping MPI build: mpic++ not in PATH. Install MS-MPI and a wrapper, or use WSL/Linux with Open MPI.
)

echo.
echo Build successful!
echo   TrafficSimulator_serial.exe  - Serial version
echo   TrafficSimulator_openmp.exe  - OpenMP parallel version
if exist TrafficSimulator_mpi.exe echo   TrafficSimulator_mpi.exe   - MPI spatial decomposition (mpirun -np N)
goto :end

:error
echo Build failed!
exit /b 1

:end
