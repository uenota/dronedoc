cmake_minimum_required(VERSION 2.8.3)
project(dronedoc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
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
