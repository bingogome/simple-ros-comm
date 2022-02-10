cmake_minimum_required(VERSION 3.8)
project(simple_ros2_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  include/simple_ros2_comm
  ${rclcpp_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} 
  src/lib/ros2_side_in.cpp
  )
add_executable(node_ros2_side_in src/nodes/node_ros2_side_in.cpp)
target_include_directories(node_ros2_side_in
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(node_ros2_side_in rclcpp std_msgs yaml-cpp)


install(TARGETS
  node_ros2_side_in
  DESTINATION lib/${PROJECT_NAME})

ament_package()