cmake_minimum_required(VERSION 3.20.0)
project(mrs_uav_coppelia_api)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_coppelia_api]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

set(DEPENDENCIES
  rclcpp
  pluginlib
  mrs_lib
  mrs_uav_hw_api
  mrs_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  MrsUavHwCoppeliaApi_Api
)

# Plugin

add_library(MrsUavHwCoppeliaApi_Api SHARED
  src/api.cpp
)

target_link_libraries(MrsUavHwCoppeliaApi_Api
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_uav_hw_api::mrs_uav_hw_api
    ${mrs_msgs_TARGETS}
    ${nav_msgss_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

target_compile_definitions(MrsUavHwCoppeliaApi_Api PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
