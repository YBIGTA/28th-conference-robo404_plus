cmake_minimum_required(VERSION 3.8)
project(yolo_jetson LANGUAGES CXX CUDA)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 53)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

set(TENSORRT_INCLUDE_DIRS
  /usr/include
  /usr/include/aarch64-linux-gnu
)

set(TENSORRT_LIBRARY_DIRS
  /usr/lib/aarch64-linux-gnu
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
)

add_executable(yolo_node
  src/ros_yolo_node.cpp
  src/yolov8.cpp
)

ament_target_dependencies(yolo_node
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
  yolo_msgs
)

target_link_directories(yolo_node PUBLIC
  ${TENSORRT_LIBRARY_DIRS}
)

target_link_libraries(yolo_node
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  nvinfer
  nvinfer_plugin
  cudart
)

if(${OpenCV_VERSION} VERSION_GREATER_EQUAL 4.7.0)
  target_compile_definitions(yolo_node PRIVATE BATCHED_NMS)
endif()

install(TARGETS
  yolo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
