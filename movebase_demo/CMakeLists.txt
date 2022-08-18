cmake_minimum_required(VERSION 3.0.2)
project(movebase_demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES movebase_learning
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(save_goal_from_rviz src/save_goal_from_rviz.cpp)
target_link_libraries(save_goal_from_rviz
  ${catkin_LIBRARIES})

#/////////////////////////////////////////////////////////////////////  
add_executable(pub_goal_to_movebase src/pub_goal_to_movebase.cpp)
target_link_libraries(pub_goal_to_movebase
  ${catkin_LIBRARIES})
  
#/////////////////////////////////////////////////////////////////////
add_executable(save_goal_from_tf_transform src/save_goal_from_tf_transform.cpp)
target_link_libraries(save_goal_from_tf_transform
  ${catkin_LIBRARIES})

#/////////////////////////////////////////////////////////////////////
add_executable(view_tf_transform src/view_tf_transform.cpp)
target_link_libraries(view_tf_transform
  ${catkin_LIBRARIES})

#/////////////////////////////////////////////////////////////////////
add_executable(pub_recharge src/pub_recharge.cpp)
target_link_libraries(pub_recharge
  ${catkin_LIBRARIES})