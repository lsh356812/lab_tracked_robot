cmake_minimum_required(VERSION 3.0.2)
project(tracked_robot)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  gas_capture
)

find_package(gazebo REQUIRED)

catkin_package(
    #INCLUDE_DIRS include
    LIBRARIES   tracked_robot_pkgs
    DEPENDS gazebo_ros roscpp rospy std_msgs gas_capture
    DEPENDS system_lib
)


include_directories(${Boost_include_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/tracked_robot_plugin.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
