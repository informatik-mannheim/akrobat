#****************************************************************************
# Copyright (C) 2021 pmdtechnologies ag
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
# PARTICULAR PURPOSE.
#
#****************************************************************************

cmake_minimum_required(VERSION 3.13)

project(royale_in_ros2)

find_package(royale REQUIRED)
link_directories(${royale_LIB_DIR})

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

# This setting causes Qt's "MOC" generation to happen automatically.
set (CMAKE_AUTOMOC ON)

# Define QT_NO_KEYWORDS
add_definitions (-DQT_NO_KEYWORDS)

include_directories ("${CMAKE_CURRENT_LIST_DIR}/include/sampleROS2"
                     "${Qt5Widgets_INCLUDE_DIRS}")

add_library(royale_nodelet SHARED
  "${CMAKE_CURRENT_SOURCE_DIR}/include/sampleROS2/RoyaleInRos.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/RoyaleInRos.cpp"
  )
target_link_libraries (royale_nodelet ${royale_LIBS})
ament_target_dependencies(royale_nodelet
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "rcutils")

add_library(royale_panel SHARED
  "${CMAKE_CURRENT_SOURCE_DIR}/include/sampleROS2/RoyaleControl.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/RoyaleControl.cpp"
  )
target_link_libraries (royale_panel Qt5::Widgets)
ament_target_dependencies(royale_panel
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "rcutils"
  "rviz_common")

add_executable (royale_in_ros2
                  "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
                  )
target_link_libraries (royale_in_ros2 royale_nodelet ${royale_LIBS})
ament_target_dependencies(royale_in_ros2
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "rcutils"
  "rviz_common")

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

install(TARGETS
  royale_nodelet
  royale_panel
  royale_in_ros2
  DESTINATION lib/${PROJECT_NAME})

install (DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install (FILES plugins.xml DESTINATION lib/${PROJECT_NAME})

ament_package()
