cmake_minimum_required(VERSION 3.14)
project(NovaEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include paths
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/usr/include/jerasure)

# Source files
file(GLOB_RECURSE SOURCES
        ${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_executable(novaengine ${SOURCES}
        include/chunk_dispatcher.hpp
        src/chunk_dispatcher.cpp
        include/rtt_monitor.hpp
        src/rtt_monitor.cpp
        include/loss_tracker.hpp
        src/loss_tracker.cpp)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(novaengine ${OpenCV_LIBS})

# FFmpeg - Enhanced linking
find_library(AVCODEC_LIB avcodec)
find_library(AVUTIL_LIB avutil)
find_library(SWSCALE_LIB swscale)
find_library(AVFORMAT_LIB avformat)
find_library(JERASURE_LIB Jerasure)
find_library(GFCOMPLETE_LIB gf_complete)

if(AVCODEC_LIB AND AVUTIL_LIB AND SWSCALE_LIB AND AVFORMAT_LIB AND JERASURE_LIB AND GFCOMPLETE_LIB)
    target_link_libraries(novaengine 
        ${AVCODEC_LIB} 
        ${AVUTIL_LIB} 
        ${SWSCALE_LIB} 
        ${AVFORMAT_LIB} 
        ${JERASURE_LIB} 
        ${GFCOMPLETE_LIB}
    )
else()
    message(FATAL_ERROR "FFmpeg or Jerasure libraries not found")
endif()

# Linux sistem linkleri
if(UNIX)
    target_link_libraries(novaengine pthread dl)
endif()

# Derlenmiş dosyayı bin klasörüne at
set_target_properties(novaengine PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)
