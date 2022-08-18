#include<ros/ros.h>
#include<signal.h>
#include<geometry_msgs/Twist.h>
#include<tf/transform_listener.h>
#include<nav_msgs/Odometry.h>
#include<string.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>  
#include<visualization_msgs/Marker.h>
#include<cmath>

#include<fstream>
#include<string>

//根据距离来发布目标位置
move_base_msgs::MoveBaseGoal goal;
//￥ 目标点与当前点之间的距离
double distance = 2;

//% 对服务进行重命名，本服务与movebase进行联系。
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
ros::Publisher cmdVelPub;
ros::Publisher marker_pub;
int file_line = 0;


void got_lines(std::string &filename,int &lines)
{
    char buffer[1024] ={0};//定义一个字节数组存放数据
    std::fstream out;
    int line = 1;
    out.open(filename,std::ios::in);
    while(!out.eof())
    {
        line+=1;
        out.getline(buffer,1024);
    }
    out.close();
    lines = line-2;
    std::cout<<"lines = "<<lines<<std::endl;
}



void init_markers(visualization_msgs::Marker *marker)
{
    marker->ns  ="waypoints";
    marker->id  =0;
    marker->type    =visualization_msgs::Marker::CUBE_LIST;
    marker->action  =visualization_msgs::Marker::ADD;
    marker->lifetime    =ros::Duration();//0 is forever
    marker->scale.x =0.2;
    marker->scale.y =0.2;
    marker->color.r = 1.0;
    marker->color.g = 0.7;
    marker->color.b = 1.0;
    marker->color.a = 1.0;

    marker->header.frame_id = "odom";
    marker->header.stamp = ros::Time::now();
}

//% action的result
void resultCb()
{
	ROS_INFO("Goal Received");
}

//% action的feedbac
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    float x, y, z;
    x = feedback->base_position.pose.position.x;
    y = feedback->base_position.pose.position.y;
    z = feedback->base_position.pose.position.z;
    
    distance = sqrt(pow((x - goal.target_pose.pose.position.x), 2) + pow((y - goal.target_pose.pose.position.y),2));
    printf("distance is = %f\r\n",distance);
    //printf("x = %f, y = %f \r\n",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
}

std::string filename = "/home/ubuntu/Desktop/pose.txt";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_move_base");
    std::string topic = "/cmd_vel";
    ros::NodeHandle node;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_marker", 10);
    ROS_INFO(">>>>>>>>>start<<<<<<<<<<");

    got_lines(filename,file_line);//获取goal数

    //￥ goal数据存到geometry_msgs::Pose数组中
    geometry_msgs::Pose pose_list[file_line];

    //￥ 正式加载数据
    char buffer[1024] ={0};
    std::fstream out;
    out.open(filename,std::ios::in);
    for(int L = 0;L<file_line;L++)
    {
        int b = 0;
        out.getline(buffer,1024);
        //￥ 解析每行数据
        for(int a=0;a<7;a++)
        {
            std::string data;
            while(buffer[b]!=',')
            {
                data += buffer[b];
                b++;
            }
            b+=1;//% 加1是隔开','
            //std::cout<<"data: "<<std::atof(data.c_str())<<std::endl;
            
            switch(a){
                case 0:
                pose_list[L].position.x = std::atof(data.c_str());
                case 1:
                pose_list[L].position.y = std::atof(data.c_str());
                case 2:
                pose_list[L].position.z = std::atof(data.c_str());
                case 3:
                pose_list[L].orientation.w = std::atof(data.c_str());
                case 4:
                pose_list[L].orientation.x = std::atof(data.c_str());
                case 5:
                pose_list[L].orientation.y = std::atof(data.c_str());
                case 6:
                pose_list[L].orientation.z = std::atof(data.c_str());
            }
        }
    }



    visualization_msgs::Marker line_list;
    init_markers(&line_list);
    
    for(int i =0;i<file_line ;i++)
    {
        line_list.points.push_back(pose_list[i].position);

    }
    ROS_INFO("waiting for move_base action server");
    
    //wait 60 seconds for the action server to beconme available
    if(!ac.waitForServer(ros::Duration(60)))
    {
        ROS_INFO("Can't connected th move base server");
        return 1;
    }
    ROS_INFO("Connected to move base server");
    ROS_INFO("Starting navigation test");

    int count = 0;
    //Cycle through the four waypoints
    while ((count<file_line) && ros::ok())
    {
       //Update the marker display
        marker_pub.publish(line_list);
       //Use the map frame to define goal poses
        goal.target_pose.header.frame_id = "map";
       //set the time stamp to now
        goal.target_pose.header.stamp = ros::Time::now();
       //set the goal pose to the i-th waypoint
        goal.target_pose.pose = pose_list[count];

       //start the robot moving toward the goal
       //send the goal pose to the movebaseaction server
        ac.sendGoal(goal, Client::SimpleDoneCallback(), &resultCb, &feedbackCb);
        printf("x = %f, y = %f\r\n",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

       //if we dont get there in time abort the goal
        ros::Duration(5).sleep();
       //$当离目标小于30cm，跳出本次规划，进入下一次规划
        while(distance >= 0.3){
            if(!ros::ok())
                break;
        }
        if(!ros::ok())
            break;
       //$ 发布下一个目标点
        count +=1;
        ROS_INFO("reday for the next goal\n");
    } 

    ROS_INFO("move_base_square.cpp end");

    
}
