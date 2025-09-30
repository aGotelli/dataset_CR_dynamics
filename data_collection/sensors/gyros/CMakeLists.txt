cmake_minimum_required(VERSION 3.10)
project(sparkfun_ism330dhcx_interface)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)

# Platform detection for informational purposes and platform-specific settings
if(WIN32)
    message(STATUS "Building for Windows platform")
    add_definitions(-D_WIN32)
elseif(UNIX AND NOT APPLE)
    message(STATUS "Building for Linux platform")
    add_definitions(-D__linux__)
else()
    message(FATAL_ERROR "Unsupported platform. Only Windows and Linux are supported.")
endif()

# Include directories - simplified without platform subdirectories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/sensor_api)
include_directories(${PROJECT_SOURCE_DIR}/include/sensor_api/st_src)

# All header files
file(GLOB ALL_H_FILES 
    ${PROJECT_SOURCE_DIR}/include/*.h
    ${PROJECT_SOURCE_DIR}/include/sensor_api/*.h
    ${PROJECT_SOURCE_DIR}/include/sensor_api/st_src/*.h
)

# All source files - no platform filtering, preprocessor directives handle platform differences
file(GLOB ALL_CPP_FILES 
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/sensor_api/*.cpp
    ${PROJECT_SOURCE_DIR}/src/sensor_api/st_src/*.c
)

message(STATUS "Header files:")
foreach(file ${ALL_H_FILES})
    message(STATUS "  ${file}")
endforeach()

message(STATUS "Source files:")
foreach(file ${ALL_CPP_FILES})
    message(STATUS "  ${file}")
endforeach()


# Find Required files
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

# Main executable
add_executable(sparkfun_ism330dhcx main.cpp ${ALL_H_FILES} ${ALL_CPP_FILES})
target_link_libraries(sparkfun_ism330dhcx ${CMAKE_THREAD_LIBS_INIT})

# TCP Server
add_executable(sparkfun_tcp_server sparkfun_tcp_server.cpp ${ALL_H_FILES} ${ALL_CPP_FILES})
target_link_libraries(sparkfun_tcp_server ${CMAKE_THREAD_LIBS_INIT})

# Optional libraries
if(Boost_FOUND)
    target_link_libraries(sparkfun_ism330dhcx ${Boost_LIBRARIES})
    target_link_libraries(sparkfun_tcp_server ${Boost_LIBRARIES})
    message(STATUS "Found Boost libraries")
else()
    message(WARNING "Boost libraries not found - building without Boost")
endif()



# Platform-specific linking
if(WIN32)
    # Windows: Link against 64-bit CH341 library (A64 suffix)
    add_library(CH341 SHARED IMPORTED)
    set_target_properties(CH341 PROPERTIES
        IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/driver/ch341_windows/CH341DLLA64.DLL"
        IMPORTED_IMPLIB "${PROJECT_SOURCE_DIR}/driver/ch341_windows/CH341DLLA64.LIB"
        INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/driver/ch341_windows/include"
    )
    target_link_libraries(sparkfun_ism330dhcx CH341)
    target_link_libraries(sparkfun_tcp_server CH341)
    
    # Add the include directory to the main target as well
    target_include_directories(sparkfun_ism330dhcx PRIVATE "${PROJECT_SOURCE_DIR}/driver/ch341_windows/include")
endif()

# Test executables
add_subdirectory(tests)