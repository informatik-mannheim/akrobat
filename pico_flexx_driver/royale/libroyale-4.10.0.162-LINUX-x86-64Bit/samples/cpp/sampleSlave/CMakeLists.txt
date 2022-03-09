cmake_minimum_required (VERSION 3.13)

set (CMAKE_PREFIX_PATH "../../../share")

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR})

project (sampleSlave)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/../../inc)

add_executable (sampleSlave sampleSlave.cpp)

copy_royale_libs (sampleSlave)

target_link_libraries (sampleSlave "${royale_LIBS}")
