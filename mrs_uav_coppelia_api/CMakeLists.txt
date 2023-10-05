cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_coppelia_api)

set(CATKIN_DEPENDENCIES
  roscpp
  cmake_modules
  mrs_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  mrs_lib
  mrs_uav_hw_api
  geometry_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIBRARIES
  MrsUavHwCoppeliaApi_Api
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

# Plugin

add_library(MrsUavHwCoppeliaApi_Api
  src/api.cpp
  )

add_dependencies(MrsUavHwCoppeliaApi_Api
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavHwCoppeliaApi_Api
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
