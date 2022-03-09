cmake_minimum_required (VERSION 3.13)

set (CMAKE_PREFIX_PATH "../../../share")

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR})

project (sampleIReplay)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/../../inc)

add_executable (sampleIReplay sampleIReplay.cpp)

copy_royale_libs (sampleIReplay)

target_link_libraries (sampleIReplay "${royale_LIBS}")
