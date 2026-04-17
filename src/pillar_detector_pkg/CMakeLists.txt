cmake_minimum_required(VERSION 3.8)
project(pillar_detector_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pillar_detector_node
  src/pillar_detector_node.cpp
  src/pillar_detector_main.cpp
)
target_include_directories(pillar_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pillar_detector_node
  rclcpp
  sensor_msgs
  std_msgs
)
if(MSVC)
  target_compile_options(pillar_detector_node PRIVATE /W4)
else()
  target_compile_options(pillar_detector_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS pillar_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
