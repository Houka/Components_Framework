cmake_minimum_required(VERSION 2.8.3)
project(components)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  cmr_lib
  cmr_hw_msgs
  controller_manager
  hardware_interface
)

FIND_PACKAGE(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

ADD_DEFINITIONS("-std=c++11")

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 #generate_messages(
  # DEPENDENCIES
  # cmr_hw_msgs
#   i2c_bus_msgs
 #)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cmr_hw
   CATKIN_DEPENDS roscpp # controller_manager
   DEPENDS i2c-tools yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${QT_USE_FILE}
  ${CMAKE_CURRENT_BINARY_DIR}
)

## Determine correct libdln library to load based on architecture.
if (CMAKE_SIZEOF_VOID_P MATCHES "8")
  set(LIBDLN /usr/lib/libdln_64.a)
else()
  set(LIBDLN /usr/lib/libdln_32.a)
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")

add_library(components
 # src/rover.cpp
  src/component.cpp
  src/ComponentGraphManager.cpp
)

add_dependencies(components
  cmr_lib
  i2c_bus_msgs_generate_messages_cpp
  cmr_hw_msgs_generate_messages_cpp
  ros_control
)

add_executable(components_test
  src/main.cpp
)

add_dependencies(components_test
  components
)
#add_dependencies(ph_sensor cmr_hw)

# ADD_DEFINITIONS(${QT_DEFINITIONS})

target_link_libraries(components
  ${LIBDLN}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  yaml-cpp
)

target_link_libraries(components_test
  components
  ${LIBDLN}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  yaml-cpp
)

#target_link_libraries(ph_sensor
#  cmr_hw
#  ${LIBDLN}
#  ${catkin_LIBRARIES}
#  ${QT_LIBRARIES}
#  yaml-cpp
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/dln_srv.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS cmr_main
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## NOTE: We may need to include other directories as we have sub directories
## inside the cmr_hw folder.

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  config
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
