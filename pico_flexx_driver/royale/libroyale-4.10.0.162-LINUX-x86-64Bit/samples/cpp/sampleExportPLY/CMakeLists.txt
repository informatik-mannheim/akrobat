cmake_minimum_required (VERSION 3.13)

set (CMAKE_PREFIX_PATH "../../../share")

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR})

project (sampleExportPLY)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/../../inc)

add_executable (sampleExportPLY sampleExportPLY.cpp)

copy_royale_libs (sampleExportPLY)

target_link_libraries (sampleExportPLY "${royale_LIBS}")
