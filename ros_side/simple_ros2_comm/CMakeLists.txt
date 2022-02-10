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
  ${YAML_CPP_INCLUDEDIR}
)

add_library(${PROJECT_NAME} 
  src/lib/ros2_side_in.cpp
  )

add_executable(node_ros2_side_in src/nodes/node_ros2_side_in.cpp)
ament_target_dependencies(node_ros2_side_in rclcpp std_msgs yaml-cpp)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()