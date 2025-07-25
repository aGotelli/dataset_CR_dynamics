cmake_minimum_required(VERSION 3.10)
project(testMark10)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable
add_executable(testMark10 testMark10.cpp)

# Platform-specific libraries
if(WIN32)
    # No additional libraries needed for Windows
else()
    # For Linux, no additional libraries needed for basic serial communication
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(testMark10 PRIVATE /W4)
else()
    target_compile_options(testMark10 PRIVATE -Wall -Wextra -O2)
endif()
