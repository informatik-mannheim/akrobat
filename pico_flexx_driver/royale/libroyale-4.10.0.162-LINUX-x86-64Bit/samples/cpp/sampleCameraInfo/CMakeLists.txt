cmake_minimum_required (VERSION 3.13)

set (CMAKE_PREFIX_PATH "../../../share")

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR})

project (sampleCameraInfo)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/../../inc)

add_executable (sampleCameraInfo sampleCameraInfo.cpp)

copy_royale_libs (sampleCameraInfo)

target_link_libraries (sampleCameraInfo "${royale_LIBS}")
