cmake_minimum_required(VERSION 2.8.3)
project(forward_kinematics)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  roslib
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen 3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES forward_kinematics
  CATKIN_DEPENDS cmake_modules roscpp roslib sensor_msgs
)

###########
## Build ##
###########

add_definitions(${Eigen_DEFINITIONS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(forward_kinematics
  src/ForwardKinematics.cpp
)

## Declare a cpp executable
add_executable(forward_kinematics_node src/main.cpp src/RosIO.cpp src/FileIO.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(forward_kinematics_node
  ${catkin_LIBRARIES} forward_kinematics
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS forward_kinematics forward_kinematics_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/plot.gp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_forward_kinematics.cpp src/FileIO.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
