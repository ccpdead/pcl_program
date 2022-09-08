#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>  

#include <tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Matrix3x3.h>

#include<cmath>

#include<fstream>
#include<string>

// movebase_global
move_base_msgs::MoveBaseGoal goal;
//￥ 目标点与当前点之间的距离
double distance = 2;
double angle = 0;

//% 对服务进行重命名，本服务与movebase进行联系。
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
ros::Publisher cmdVelPub;
int file_line = 0;


//$ 获取保存地址文件的行数
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

//￥ action的result
void resultCb()
{
	ROS_INFO("Goal Received");
}

//￥ action的feedback
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    float x, y;
    x = feedback->base_position.pose.position.x;
    y = feedback->base_position.pose.position.y;

    tf2::Quaternion quation1,quation2;
    quation1.setW(feedback->base_position.pose.orientation.w);
    quation1.setX(feedback->base_position.pose.orientation.x);
    quation1.setY(feedback->base_position.pose.orientation.y);
    quation1.setZ(feedback->base_position.pose.orientation.z);

    quation2.setW(goal.target_pose.pose.orientation.w);
    quation2.setX(goal.target_pose.pose.orientation.x);
    quation2.setY(goal.target_pose.pose.orientation.y);
    quation2.setZ(goal.target_pose.pose.orientation.z);

    double roll1 = 0,pitch1 = 0,yaw1 = 0;
    double roll2 = 0,pitch2 = 0,yaw2 = 0;

    tf2::Matrix3x3 m1(quation1);
    tf2::Matrix3x3 m2(quation2);

    m1.getRPY(roll1,pitch1,yaw1);
    m2.getRPY(roll2,pitch2,yaw2);

    distance = sqrt(pow((x - goal.target_pose.pose.position.x), 2) + pow((y - goal.target_pose.pose.position.y),2));
    angle = fabs(yaw1 - yaw2);
    printf("distance = %f\r\n",distance);
    printf("angle = %f\r\n",angle);
    //printf("x = %f, y = %f \r\n",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    // printf("roll1 :%f,pitch1 :%f,yaw1 :%f",roll1,pitch1,yaw1);
    // printf("roll2 :%f,pitch2 :%f,yaw2 :%f",roll2,pitch2,yaw2);
}

//￥ 导航坐标点的名称
std::string filename = "/home/ubuntu/Desktop/pose.txt";


//￥ main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_move_base");
    std::string topic = "/cmd_vel";
    ros::NodeHandle node;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    
    ROS_INFO(">>>>>>>>>start<<<<<<<<<<");

    got_lines(filename,file_line);//获取goal数

    //￥ goal数据存到geometry_msgs::Pose数组中
    geometry_msgs::Pose pose_list[file_line];

    //￥ 加载导航数据点
    char buffer[1024] ={0};
    std::fstream out;
    out.open(filename,std::ios::in);

    //￥ 将坐标点加载到 pose_list
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
    //$ 依次将导航坐标点发布给 move_base
    while ((count<file_line) && ros::ok())
    {
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
        while(distance >= 0.3 || angle >=1.57){
            if(!ros::ok())
                break;
        }
        if(!ros::ok())
            break;
       //$ 发布下一个目标点
        count +=1;
        //循环发送目标
        if(count == file_line){
            count = 0;
        }
        ROS_INFO("reday for the next goal\n");
    } 

    ROS_INFO("move_base_square.cpp end");

    
}
