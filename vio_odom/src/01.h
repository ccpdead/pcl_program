#ifndef __01_H
#define __01_H

#include<iostream>
#include"nav_msgs/Odometry.h"
#include"ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
    class vio_odom
    {
        public:
            //构造函数
            vio_odom(ros::NodeHandle* nodehandle);
        private:
            //初始化指针
            ros::NodeHandle nh_;
            ros::Subscriber vio_odom_sub_;
            ros::Publisher vio_odom_pub_;

            //定义
            void init_sub();
            void init_pub();
            void Callback(const nav_msgs::Odometry& lidar_odom);

            //初始化数据类型
            nav_msgs::Odometry odom_data;
            Eigen::Quaternionf quaternion2;
            
    };

#endif