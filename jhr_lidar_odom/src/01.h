#ifndef __01_H
#define __01_H

#include <iostream>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

class lidar_odom
{
    public:
        //构造函数
        lidar_odom(ros::NodeHandle* nodehandle);
    
    private:
        //初始化指针
        ros::NodeHandle nh_;
        ros::Subscriber  lidar_odom_sub_;
        ros::Publisher lidar_odom_pub_;

        //定义
        void init_sub();
        void init_pub();
        void Callback(const nav_msgs::Odometry& lidar_odom);

        //初始化数据类型
        nav_msgs::Odometry odom_data;
};

#endif