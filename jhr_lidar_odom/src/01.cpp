#include "01.h"
#include <math.h>

lidar_odom::lidar_odom(ros::NodeHandle *nodehandle) : nh_(*nodehandle) //构造函数
{
    init_sub();
    init_pub();
}
//初始化发布者
void lidar_odom::init_pub()
{
    ROS_INFO("init the publisher\n");
    lidar_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/hector/odom_covariance", 1, true);
}

//初始化订阅者
void lidar_odom::init_sub()
{
    ROS_INFO("init the subscriber\n");
    lidar_odom_sub_ = nh_.subscribe("/hector/scanmatch_odom", 1, &lidar_odom::Callback, this);
}

//初始化回调函数
void lidar_odom::Callback(const nav_msgs::Odometry &odom)
{
    const float covariance[36] = {0.001, 0, 0, 0, 0, 0,
                                  0, 0.01, 0, 0, 0, 0,
                                  0, 0, 1e6, 0, 0, 0,
                                  0, 0, 0, 1e6, 0, 0,
                                  0, 0, 0, 0, 1e6, 0,
                                  0, 0, 0, 0, 0, 1e3};
    odom_data = odom;
    odom_data.pose.covariance = {0};
    for (int i = 0; i < 36; i++)
    {
        odom_data.pose.covariance[i] = covariance[i];
    }
    lidar_odom_pub_.publish(odom_data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_odom");
    ros::NodeHandle nh;
    ROS_INFO("main is ok!!!");
    lidar_odom lidar_odom_class(&nh);

    ROS_INFO("let the callback do all the work");
    ros::spin();
    return 0;
}
