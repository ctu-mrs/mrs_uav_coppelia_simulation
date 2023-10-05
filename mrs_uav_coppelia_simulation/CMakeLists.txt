cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_coppelia_simulation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(DIRECTORY config coppelia_resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tmux
  )
