cmake_minimum_required(VERSION 2.8.3)
project(gtsam_ros)

## List ROS libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)

## System dependencies
find_package(Eigen REQUIRED)
find_package(GTSAM REQUIRED)
set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam-shared)

## The catkin_package exports
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gtsam_ros
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen GTSAM
)

## Include Directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

## Libraries
add_library(gtsam_ros
  include/gtsam_ros/gtsam_gtest.h 
  include/gtsam_ros/gtsam_ros.h 
  include/gtsam_ros/gtsam_serialization.h 
  src/gtsam_ros/gtsam_serialization.cpp
)
target_link_libraries(gtsam_ros
  ${catkin_LIBRARIES}
  ${GTSAM_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS gtsam_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/gtsam_ros/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(gtsam_ros-test test/test_gtsam_ros.cpp)
if(TARGET gtsam_ros-test)
  target_link_libraries(gtsam_ros-test 
  	gtsam_ros
  )
endif()
