#include<ros/ros.h>
#include<iostream>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<std_msgs/String.h>
#include<fstream>

using namespace std;
/*
￥通过rviz获取移动机器人的目标位置信息.
*/

ofstream in;

void Feedback(const geometry_msgs::PoseStamped::ConstPtr& feedback)
{
    cout<<"<<<Received>>>"<<endl;
    float data[7] = {0};
    data[0] = feedback->pose.position.x;
    data[1] = feedback->pose.position.y;
    data[2] = feedback->pose.position.z;

    data[3] = feedback->pose.orientation.w;
    data[4] = feedback->pose.orientation.x;
    data[5] = feedback->pose.orientation.y;
    data[6] = feedback->pose.orientation.z;

    for(int i = 0;i<7;i++){
        if(i!=6){
            in<<data[i]<<",";
        }else{
            in<<data[i]<<"\n";
        }
    }
}

int main(int argc, char** argv)
{

    in.open("/home/ubuntu/Desktop/pose.txt",ios::trunc);
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;
    //$ 收到rviz上的发布的机器人导航目标后进入回调函数，经目标保存在文件中
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 1000, Feedback);
    cout<<"Read for saving......"<<endl;
    while(ros::ok())
    {
        ros::spinOnce();
    }
    in.close();
    cout<<"demo stopped.................."<<std::endl;
    return 0;
}