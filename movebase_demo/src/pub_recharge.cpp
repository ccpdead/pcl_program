#include<ros/ros.h>
#include<geometry_msgs/Twist.h>//cmd_vel头文件
#include<tf/transform_listener.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>  
#include<tf2/LinearMath/Quaternion.h>

#include<string>
#include<cmath>


/*
    ￥发布充电桩位置信息
*/
//////////////////////////////////////////////////////////////////////////////////
//--------------------------------------typedef 相当与另外起名字.-----------------------------------
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
//////////////////////////////////////////////////////////////////////////////////

//本数字表示总共的坐标位置数
int number_position = 1;
//动作成功后进入到此函数中
void activeCb(){
	ROS_INFO("Goal Received");
}

//服务运行状态回调函数
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
    float x, y, z;
    x = feedback->base_position.pose.position.x;
    y = feedback->base_position.pose.position.y;
    ROS_INFO("x= %f, y = %f",x,y);
}

int 
main(int argc, char **argv){
    //----------------------------------------------------
    float distance = 1;//存放当前机器人与充电桩的距离.
    float angle = 0;//存放角度
    tf::Quaternion quate;
    bool zero_angle = false;
    //----------------------------------------------------
    ros::init(argc, argv, "nav_move_base");

    std::string topic = "/cmd_vel";
    ros::NodeHandle node;
    ros::Publisher cmdVelPub;
    geometry_msgs::Twist cmd_vel_param;
    cmd_vel_param.linear.x = 0.0;
    cmd_vel_param.linear.y = 0.0;
    cmd_vel_param.angular.z = 0.0;
    tf::TransformListener listener;
    tf::StampedTransform transform;//用于存放转换后的关系.
    move_base_msgs::MoveBaseGoal goal;


    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);//Subscribe to the move base action server;
    cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 5);//publisher to manually control the robot
    ROS_INFO("move_base_square.cpp start...........");//a pose consition of a position and orientation in the map frame;
    double set_goal[number_position][7] = {-0.2000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000};
    
    geometry_msgs::Pose pose_list[number_position];//定义变量来储存目标位置信息
    for(int i = 0;i<number_position;i++)
    {
        pose_list[i].position.x = set_goal[i][0];
        pose_list[i].position.y = set_goal[i][1];
        pose_list[i].position.z = set_goal[i][2];

        pose_list[i].orientation.w = set_goal[i][3];
        pose_list[i].orientation.x = set_goal[i][4];
        pose_list[i].orientation.y = set_goal[i][5];
        pose_list[i].orientation.z = set_goal[i][6];

    }

    ROS_INFO("waiting for move_base action server");
    if(!ac.waitForServer(ros::Duration(60)))//wait 60 seconds for the action server to beconme available
    {
        ROS_INFO("Can't connected th move base server");
        return -1;
    }
    ROS_INFO("Starting navigation test");

    
    goal.target_pose.header.frame_id = "odom_combined";//Use the map frame to define goal poses
    goal.target_pose.header.stamp = ros::Time::now();//set the time stamp to now
    goal.target_pose.pose = pose_list[0];//set the goal pose to the i-th waypoint
    ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);//send the goal pose to the movebaseaction server
    bool finished_within_time = ac.waitForResult(ros::Duration(180));//waiting for 180s

    if(!finished_within_time)//if we dont get there in time abort the goal
    {
        ac.cancelGoal();//取消下一个目标点.
        ROS_INFO("Time out achieving goal in 180's...............");
    }else{
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {ROS_INFO("Goal succeeded..................");}
        else {ROS_INFO("failed beacuse some thing............");}
    }
    ROS_INFO("reday for the next goal\n"); 

    ros::Rate r(20);//10Hz
    while(ros::ok()){
        try{
        listener.waitForTransform("/odom_combined",
                                    "base_footprint",
                                    ros::Time(0),ros::Duration(0.5));
        listener.lookupTransform("/odom_combined",
                                    "/base_footprint",
                                    ros::Time(0),transform);
        }catch(tf::TransformException& ex){
                ROS_ERROR("%s",ex.what());
                return -1;
        }
        distance = transform.getOrigin().x();//获取x坐标
        quate = transform.getRotation();//获取四元数
        double roll, pitch, yaw;
        tf::Matrix3x3(quate).getRPY(roll, pitch, yaw);//四元数转化为欧拉角.

        printf("yaw is %f\r\n",yaw);
        printf("distance is %f\r\n",distance);
        printf("\r\n");
        if(!zero_angle){
            printf("angle is goon.................\r\n");
            if(yaw<-0.01){
                cmd_vel_param.linear.x = 0.0;
                cmd_vel_param.linear.y = 0.0;
                cmd_vel_param.angular.z = 0.1;
                cmdVelPub.publish(cmd_vel_param);
            }else if(yaw > 0.01){
                cmd_vel_param.linear.x = 0.0;
                cmd_vel_param.linear.y = 0.0;
                cmd_vel_param.angular.z = -0.1;
                cmdVelPub.publish(cmd_vel_param);
            }else{
                cmd_vel_param.linear.x = 0.05;
                cmd_vel_param.linear.y = 0.0;
                cmd_vel_param.angular.z = 0.0;
                cmdVelPub.publish(cmd_vel_param);
                zero_angle = true;
                printf("angle is zero........\r\n");
                }
        }else{
            printf("linear is goon..................\r\n");
            if(distance<-0.15){
                cmd_vel_param.linear.x = 0.05;
                cmd_vel_param.linear.y = 0.0;
                cmd_vel_param.angular.z = 0.0;
                cmdVelPub.publish(cmd_vel_param);
                
            }else if(distance>-0.075 && (yaw <-0.01 || yaw > 0.01)){
                zero_angle = false;
            }else if(distance > -0.015 && (yaw>-0.01 || yaw < 0.01)){
                cmd_vel_param.linear.x = 0.0;
                cmd_vel_param.linear.y = 0.0;
                cmd_vel_param.angular.z = 0.0;
                cmdVelPub.publish(cmd_vel_param);

                printf("recved the recharge...............\r\n");
                ros::shutdown();
            }
        }
        r.sleep();
    }

    
}
