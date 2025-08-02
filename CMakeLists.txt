cmake_minimum_required(VERSION 3.14)
project(NovaEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler optimizations for ultra-low latency
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mtune=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -funroll-loops")

# Include paths
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/usr/include/jerasure)
include_directories(/usr/local/include)

# Source files
file(GLOB_RECURSE SOURCES
        ${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_executable(novaengine ${SOURCES})

# OpenCV - Enhanced video processing
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(novaengine ${OpenCV_LIBS})

# FFmpeg - Complete multimedia stack
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED 
    libavcodec 
    libavutil 
    libswscale 
    libavformat 
    libswresample
)

include_directories(${FFMPEG_INCLUDE_DIRS})
target_link_libraries(novaengine ${FFMPEG_LIBRARIES})

# Jerasure - Reed-Solomon FEC
find_library(JERASURE_LIB NAMES Jerasure jerasure)
find_library(GFCOMPLETE_LIB NAMES gf_complete gfcomplete)
find_library(REED_SOL_LIB NAMES reed_sol)

if(JERASURE_LIB AND GFCOMPLETE_LIB)
    target_link_libraries(novaengine 
        ${JERASURE_LIB} 
        ${GFCOMPLETE_LIB}
    )
    add_definitions(-DHAVE_JERASURE)
    message(STATUS "Jerasure libraries found")
else()
    message(FATAL_ERROR "Jerasure libraries not found. Install: sudo apt-get install libjerasure-dev")
endif()

# System libraries for networking and threading
if(UNIX)
    target_link_libraries(novaengine 
        pthread 
        dl 
        rt
    )
endif()

# Runtime output directory
set_target_properties(novaengine PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)

# Print configuration summary
message(STATUS "NovaEngine Configuration:")
message(STATUS "  - OpenCV: ${OpenCV_VERSION}")
message(STATUS "  - FFmpeg: ${FFMPEG_VERSION}")
message(STATUS "  - Jerasure: Found")
message(STATUS "  - C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  - Optimization: -O3 -march=native")
