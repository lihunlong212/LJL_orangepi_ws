cmake_minimum_required(VERSION 3.8)
project(drone_camera_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(drone_camera_node src/drone_camera_node.cpp)
ament_target_dependencies(drone_camera_node rclcpp std_msgs)
target_include_directories(drone_camera_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(drone_camera_node ${OpenCV_LIBS})

install(TARGETS
  drone_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
