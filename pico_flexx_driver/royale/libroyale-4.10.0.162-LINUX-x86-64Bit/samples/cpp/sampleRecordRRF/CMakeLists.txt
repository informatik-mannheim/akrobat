cmake_minimum_required (VERSION 3.13)

set (CMAKE_PREFIX_PATH "../../../share")

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR})

project (sampleRecordRRF)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/../../inc)

add_executable (sampleRecordRRF sampleRecordRRF.cpp)

copy_royale_libs (sampleRecordRRF)

target_link_libraries (sampleRecordRRF "${royale_LIBS}")
