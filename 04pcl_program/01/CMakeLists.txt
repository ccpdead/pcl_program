cmake_minimum_required(VERSION 3.0.2)
project(01)

find_package(PCL REQUIRED
PATHS ~/pcl-19/install
NO_DEFAULT_PATH)

find_package(catkin REQUIRED COMPONENTS
geometry_msgs
move_base_msgs
roscpp
)
#########################################################
find_package(cmake_modules REQUIRED)

# 添加eigen3头文件
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# 设置bin可执行二进制文件的存放位置
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
catkin_package(
)

# 设置加载ros的头文件地址
include_directories(
    ${catkin_INCLUDE_DIRS}
)
####################################################
# 设置PCL的include头文件
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
################################################3

#订阅pointcloud2点云信息并转化为PCL格式,进行滤波处理后重新转化为ROS信息格式并发布
# add_executable (main src/03.cpp)
# target_link_libraries (main 
#                             ${PCL_LIBRARIES}
#                             ${catkin_LIBRARIES})

#保存点云信息
add_executable (save_pcd src/04.cpp)
target_link_libraries (save_pcd 
                            ${PCL_LIBRARIES}
                            ${catkin_LIBRARIES})
#显示点云信息,用于测试阶段
# add_executable (pcl_viewer src/04-2.cpp)
# target_link_libraries (pcl_viewer 
#                             ${PCL_LIBRARIES}
#                             ${catkin_LIBRARIES})

add_executable (recharge_test1 src/05.cpp)
target_link_libraries (recharge_test1 
                            ${PCL_LIBRARIES}
                            ${catkin_LIBRARIES})
                            
#识别充电桩,并将其想对于相机的坐标表示出来.
add_executable (recharge_test2 src/05-2.cpp)
target_link_libraries (recharge_test2 
                            ${PCL_LIBRARIES}
                            ${catkin_LIBRARIES})

add_executable (recharge_test3 src/05-3.cpp)
target_link_libraries (recharge_test3 
                            ${PCL_LIBRARIES}
                            ${catkin_LIBRARIES})

#采用TOF相机实现充电桩识别定位
add_executable (recharge_test4 src/05-2-tof.cpp)
target_link_libraries (recharge_test4 
                            ${PCL_LIBRARIES}
                            ${catkin_LIBRARIES})