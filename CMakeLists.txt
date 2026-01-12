cmake_minimum_required(VERSION 2.8.3)
project(apriltag_navigation)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf
)

## Python setup
catkin_python_setup()

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf
)

###########
## Build ##
###########

## No C++ components to build (pure Python package)

#############
## Install ##
#############

## Install Python package
catkin_install_python(PROGRAMS
  nodes/main_node.py
  scripts/diagnostics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Install Python source modules
install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)
