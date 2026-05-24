cmake_minimum_required(VERSION 3.5)
project(yolo_bridge)

# 1. 基础标准设置
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 2. 寻找必要依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED) # YOLO 必须用到 OpenCV

# 3. 【关键】尝试寻找 CUDA (用于区分 VM 和 Nano)
find_package(CUDA QUIET)

if(CUDA_FOUND)
    message(STATUS "CUDA Found! TensorRT Mode Enabled.")
    add_definitions(-DUSE_CUDA) # 给代码传一个宏，方便你在代码里写 #ifdef USE_CUDA
    
    # Jetson Nano 的 TensorRT 库路径（通常在这里）
    set(TENSORRT_LIBS nvinfer nvinfer_plugin)
    include_directories(${CUDA_INCLUDE_DIRS})
else()
    message(WARNING "CUDA NOT FOUND! Compiling in Simulation Mode (No TensorRT).")
endif()

# 4. 设置头文件包含路径
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 5. 定义可执行文件 (把你搬过去的 cpp 全写在这里)
add_executable(yolo_node 
  src/main.cpp 
  src/yolov8.cpp
)

# 6. 链接 ROS 2 的库
ament_target_dependencies(yolo_node
  rclcpp
  vision_msgs
  geometry_msgs
  cv_bridge
  sensor_msgs
  std_msgs
)

# 7. 链接 OpenCV 和 CUDA/TensorRT 的库
target_link_libraries(yolo_node
  ${OpenCV_LIBS}
)

if(CUDA_FOUND)
    target_link_libraries(yolo_node ${CUDA_LIBRARIES} ${TENSORRT_LIBS})
endif()

# 8. 安装配置 (方便之后用 ros2 run 命令)
install(TARGETS
  yolo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()