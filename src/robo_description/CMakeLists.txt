cmake_minimum_required(VERSION 3.8)
project(robo_description)

find_package(ament_cmake REQUIRED)

# Install all simulation/description assets into the package share directory.
install(
  DIRECTORY urdf launch worlds models models_yolo rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the desktop-sim nodes/scripts as executables. track_path.py and
# gen_world.py are imported by demo_orchestrator at runtime, so they live in
# the same lib dir.
install(
  PROGRAMS
    scripts/sim_yolo_node.py
    scripts/demo_orchestrator.py
    scripts/gen_world.py
    scripts/track_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
