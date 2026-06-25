cmake_minimum_required(VERSION 3.8)
project(csi_camera)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(csi_camera_node
  src/csi_camera_node.cpp
)

target_include_directories(csi_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(csi_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(csi_camera_node
  ${OpenCV_LIBS}
)

install(TARGETS
  csi_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
