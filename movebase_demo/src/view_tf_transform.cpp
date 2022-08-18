#include<ros/ros.h>
#include<iostream>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_listener.h>
#include<std_msgs/String.h>
using namespace std;

FILE *fp;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;
    
    char input;
    geometry_msgs::PoseStamped::ConstPtr feedback;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while(ros::ok())
    {

        static int times = 0;
        times++;
        printf("saved times %d\r\n",times);
        cin>>input;   
        if(input == 's')
        ros::spinOnce();
        else if(input == 'q'){break;}
        //////////////////////////////////////////////////////
        cout<<"I heard"<<endl;
        float data[7] = {0};

        try
        {
            listener.waitForTransform("/map",
                                        "/base_footprint",
                                        ros::Time(0),ros::Duration(0.5));
            listener.lookupTransform("/map",
                                        "/base_footprint",
                                    ros::Time(0),transform);
        }catch(tf::TransformException& ex){
            ROS_ERROR("%s",ex.what());
            return -1;
        }
        data[0] = transform.getOrigin().x();
        data[1] = transform.getOrigin().y();
        data[2] = transform.getOrigin().z();

        data[3] = transform.getRotation().w();
        data[4] = transform.getRotation().x();
        data[5] = transform.getRotation().y();
        data[6] = transform.getRotation().z();
        
        printf("x=%.2f,y=%.2f,z=%.2f,w=%.2f,x=%.2f,y=%.2f,z=%.2f\r\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);

    }

    return 0;
}