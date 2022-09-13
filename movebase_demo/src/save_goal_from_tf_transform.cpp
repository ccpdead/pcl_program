#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <std_msgs/String.h>
using namespace std;
/*
    ￥根据TF转换来获取机器人的目标位置
*/

ofstream in;
int main(int argc, char **argv)
{
    // char type;
    // cout<<"input the type"<<endl;
    // cin>>type;
    // if(type == 'r')
    // {
    // printf("start saving recharge transofrm\r\n");
    // in.open("/home/ubuntu/Desktop/pose_recharge.txt",ios::trunc);
    // }else
    // {
    printf("start saving movebase transofrm\r\n");
    in.open("/home/ubuntu/Desktop/pose.txt", ios::trunc);
    // }

    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;

    char input;
    geometry_msgs::PoseStamped::ConstPtr feedback;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while (ros::ok())
    {

        static int times = 0;
        times++;
        printf("saved times %d\r\n", times);
        cin >> input;
        if (input == 's')
            ros::spinOnce();
        else if (input == 'q')
        {
            break;
        }
        //////////////////////////////////////////////////////
        cout << "I heard" << endl;
        float data[7] = {0};
        // //￥ 充电模式
        // if (type == 'r') //若为r,则为充电模式
        // {
        //     try
        //     {
        //         listener.waitForTransform("/odom_combined",
        //                                   "/base_footprint",
        //                                   ros::Time(0), ros::Duration(0.5));
        //         listener.lookupTransform("/odom_combined",
        //                                  "/base_footprint",
        //                                  ros::Time(0), transform);
        //     }
        //     catch (tf::TransformException &ex)
        //     {
        //         ROS_ERROR("%s", ex.what());
        //         return -1;
        //     }
        // }
        // else
        //$ 导航模式
        // {
        try
        {
            listener.waitForTransform("/map",
                                      "/base_footprint",
                                      ros::Time(0), ros::Duration(0.5));
            listener.lookupTransform("/map",
                                     "/base_footprint",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return -1;
        }
        // }
        data[0] = transform.getOrigin().x();
        data[1] = transform.getOrigin().y();
        data[2] = transform.getOrigin().z();

        data[3] = transform.getRotation().w();
        data[4] = transform.getRotation().x();
        data[5] = transform.getRotation().y();
        data[6] = transform.getRotation().z();
        //$将数据写入txt文件
        for (int i = 0; i < 7; i++)
        {
            if (i != 6)
            {
                in << data[i] << ",";
            }
            else
            {
                in << data[i] << "\n";
            }
        }
    }
    in.close();
    cout << ".....................pose saved........................." << std::endl;
    return 0;
}