#include"nodeletclass1.h"
#include<pluginlib/class_list_macros.h>
#include<ros/ros.h>

nodeletclass1::nodeletclass1()
{


}
void nodeletclass1::onInit()
{

    //输出信息
    NODELET_DEBUG("Init nodelet...........");
    ROS_INFO("Nodelet is OK for test_____________");
}

//$nodelet 本质是把节点当作插件来使用，需要调用PLUGINLIB的宏定义。
//第一个参数是类名，第二个参数是父类
PLUGINLIB_EXPORT_CLASS(nodeletclass1, nodelet::Nodelet);