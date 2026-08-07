cmake_minimum_required(VERSION 3.20.0)
project(mrs_uav_coppelia_simulation)

find_package(ament_cmake REQUIRED)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(DIRECTORY config coppelia_resources
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}/tmux
)

ament_package()
