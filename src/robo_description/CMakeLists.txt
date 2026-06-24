cmake_minimum_required(VERSION 3.8)
project(robo_description)

find_package(ament_cmake REQUIRED)

# Install all simulation/description assets into the package share directory.
install(
  DIRECTORY urdf launch config worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
