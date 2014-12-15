cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#//--------------------AKROBAT------------------------//
rosbuild_add_library(akrobat_init library/akrobat_init.cpp)
rosbuild_add_executable(commander src/commander.cpp)
rosbuild_add_executable(akrobat src/akrobat_main.cpp)
target_link_libraries(akrobat akrobat_init)
#//------------------END AKROBAT----------------------//

#//--------------------AKROBAT_V1---------------------//
#rosbuild_add_library(akrobat_init_v1 library/akrobat_init_v1.cpp)
#rosbuild_add_executable(akrobat src/akrobat_mainv1.cpp)
#rosbuild_add_executable(aktf src/akrobatTree.cpp)
#rosbuild_add_executable(aklist src/aklistener.cpp)
#rosbuild_add_executable(tf_zks_kks src/zks_kks.cpp)
#rosbuild_add_executable(tf_kks_bks src/kks_bks.cpp)
#rosbuild_add_executable(tf_zks_kks src/tfp1.cpp)
#rosbuild_add_executable(tf_kks_bks src/tfp2.cpp)
#target_link_libraries(akrobat akrobat_init_v1)
#//------------------END AKROBAT_V1-------------------//
