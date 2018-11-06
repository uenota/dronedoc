cmake_minimum_required(VERSION 2.8.3)

project(dronedoc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
  nav_msgs
  tf
)

find_package(gazebo REQUIRED)
if("${GAZEBO_VERSION}" VERSION_LESS "7.4")
  message(FATAL_ERROR "Gazebo 7.4 or later is required.")
endif()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_sample src/offboard_sample.cpp)
target_link_libraries(offboard_sample ${catkin_LIBRARIES})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

set(PACK_AIRFRAME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/posix_airframes)
set(LOCAL_AIRFRAME_DIR $ENV{HOME}/.ros/etc/init.d-posix)

if(NOT EXISTS ${LOCAL_AIRFRAME_DIR})
  message(FATAL_ERROR "PX4 Firmware is REQUIRED.\nInstall and run 'make posix_sitl_default gazebo'.\nIf this persists after installation, make symbolic link from Firmware/ROMFS/px4fmu_common to ~/.ros/etc")
endif()

add_custom_target(iris_2d_lidar
                  ALL ln -s -b ${PACK_AIRFRAME_DIR}/70010_iris_2d_lidar ${LOCAL_AIRFRAME_DIR}/)