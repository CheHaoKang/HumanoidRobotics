cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  roslib
)

## System dependencies are found with CMake's conventions
find_package(Eigen 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS random)

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
  LIBRARIES particle_filter
  CATKIN_DEPENDS cmake_modules roscpp roslib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
## Declare a cpp library
add_library(particle_filter
  src/ParticleFilter.cpp
)

## Declare a cpp executable
add_executable(particle_filter_node src/main.cpp src/FileIO.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(particle_filter_node
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} particle_filter
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS particle_filter particle_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(particle_filter-test test/test_particle_filter.cpp)
if(TARGET particle_filter-test)
  target_link_libraries(particle_filter-test particle_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()
