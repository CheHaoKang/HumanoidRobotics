cmake_minimum_required(VERSION 2.8.3)
project(projective_geometry)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
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
  LIBRARIES projective_geometry
  CATKIN_DEPENDS cmake_modules roscpp
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
add_library(projective_geometry
  src/ProjectiveGeometry.cpp
)

## Declare a cpp executable
add_executable(projective_geometry_node src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(projective_geometry_node
  ${catkin_LIBRARIES} projective_geometry
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS projective_geometry projective_geometry_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_projective_geometry.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()


