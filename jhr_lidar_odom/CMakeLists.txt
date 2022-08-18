cmake_minimum_required(VERSION 3.0.2)
project(jhr_lidar_odom)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jhr_lidar_odom
#  CATKIN_DEPENDS message_generation nav_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(jhr_lidar_odom src/01.cpp)
target_link_libraries(jhr_lidar_odom 
${catkin_LIBRARIES})

