cmake_minimum_required (VERSION 3.13)

set (CMAKE_PREFIX_PATH "../../../share")

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR})

project (sampleRetrieveData)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/../../inc)

add_executable (sampleRetrieveData sampleRetrieveData.cpp)

copy_royale_libs (sampleRetrieveData)

target_link_libraries (sampleRetrieveData "${royale_LIBS}")
