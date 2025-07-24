@echo off
echo Compiling testMark10.cpp...

REM Using g++ (MinGW) - uncomment if you have MinGW installed
REM g++ -std=c++11 -O2 -Wall -o testMark10.exe testMark10.cpp

REM Using MSVC - uncomment if you have Visual Studio installed
REM cl /EHsc /O2 testMark10.cpp /Fe:testMark10.exe

REM Using CMake (recommended)
if not exist build mkdir build
cd build
cmake ..
cmake --build . --config Release
cd ..

echo.
echo Compilation complete! Run testMark10.exe to test.
pause
