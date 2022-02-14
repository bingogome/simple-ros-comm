cmake_minimum_required(VERSION 3.5)
project(simple_ros_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  roslaunch
  roslib
  )

find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include/simple_ros_comm include 
  LIBRARIES simple_ros_comm
  CATKIN_DEPENDS roscpp
  DEPENDS YAML_CPP
  )

add_library(${PROJECT_NAME} 
  src/lib/ros_side_in.cpp
  src/lib/ros_side_out.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC 
  include/simple_ros_comm
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_ros_side_in src/nodes/node_ros_side_in.cpp)
target_link_libraries(node_ros_side_in ${PROJECT_NAME})

add_executable(node_ros_side_out src/nodes/node_ros_side_out.cpp)
target_link_libraries(node_ros_side_out ${PROJECT_NAME})

add_executable(node_test_ros_side_out src/nodes/node_test_ros_side_out.cpp)
target_link_libraries(node_test_ros_side_out ${PROJECT_NAME})
