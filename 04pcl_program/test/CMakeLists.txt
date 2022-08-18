cmake_minimum_required(VERSION 3.0.2)
project(test)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)
find_package(PCL REQUIRED
PATHS ~/pcl-19/install
NO_DEFAULT_PATH)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

####################################################
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
################################################3
add_executable (test_1 src/01.cpp)
target_link_libraries (test_1 
                            ${PCL_LIBRARIES}
                            ${catkin_LIBRARIES})

