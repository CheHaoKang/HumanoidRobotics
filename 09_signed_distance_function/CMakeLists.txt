cmake_minimum_required(VERSION 2.8.3)
project(signed_distance_function)
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
  LIBRARIES signed_distance_function
  CATKIN_DEPENDS cmake_modules roscpp roslib
)

###########
## Build ##
###########

add_definitions(${Eigen_DEFINITIONS})

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(signed_distance_function
  src/SignedDistanceFunction.cpp
)

# Declare a cpp executable
add_executable(signed_distance_function_node src/main.cpp src/FileIO.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(signed_distance_function_node
  ${catkin_LIBRARIES} signed_distance_function
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS signed_distance_function signed_distance_function_node
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

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_signed_distance_function.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()


