#include"01.h"
#include<math.h>


//构造函数
vio_odom::vio_odom(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    init_sub();
    init_pub();
}

//初始化发布者
void vio_odom::init_pub()
{
    ROS_INFO("init the publisher\n");
    vio_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/processed_odom", 1, true);
}

//初始化订阅者
void vio_odom::init_sub()
{
    ROS_INFO("init the subscriber\n");
    vio_odom_sub_ = nh_.subscribe("/rtabmap/vio_odom",1,&vio_odom::Callback, this);
}

//初始化回调函数
void vio_odom::Callback(const nav_msgs::Odometry& odom)
{
    odom_data = odom;
    odom_data.pose.pose.position.z = 0;
    Eigen::Quaternionf quaternion(odom_data.pose.pose.orientation.w,
                                odom_data.pose.pose.orientation.x,
                                odom_data.pose.pose.orientation.y,
                                odom_data.pose.pose.orientation.z);
    Eigen::Vector3f eulerAngle = quaternion.matrix().eulerAngles(2,1,0);
    eulerAngle.x() = 0;
    eulerAngle.y() = 0;
    quaternion2 = Eigen::AngleAxisf(eulerAngle[0],Eigen::Vector3f::UnitZ())*
                  Eigen::AngleAxisf(eulerAngle[1],Eigen::Vector3f::UnitZ())*
                  Eigen::AngleAxisf(eulerAngle[2],Eigen::Vector3f::UnitZ());
    odom_data.pose.pose.orientation.w = quaternion2.w();
    odom_data.pose.pose.orientation.x = quaternion2.x();
    odom_data.pose.pose.orientation.y = quaternion2.y();
    odom_data.pose.pose.orientation.z = quaternion2.z();

    vio_odom_pub_.publish(odom_data);//发布里程计数据
    ////////////////////////////////////////////////////////////////////

    static tf2_ros::TransformBroadcaster br;//初始化tf广播
    geometry_msgs::TransformStamped TransformStamped;
    TransformStamped.header.stamp = ros::Time::now();
    TransformStamped.header.frame_id = "processed_tf";
    TransformStamped.child_frame_id = "camera_link";
    TransformStamped.transform.translation.x = odom_data.pose.pose.position.x;
    TransformStamped.transform.translation.y = odom_data.pose.pose.position.y;
    TransformStamped.transform.translation.z = odom_data.pose.pose.position.z;
    TransformStamped.transform.rotation.w = odom_data.pose.pose.orientation.w;
    TransformStamped.transform.rotation.x = odom_data.pose.pose.orientation.x;
    TransformStamped.transform.rotation.y = odom_data.pose.pose.orientation.y;
    TransformStamped.transform.rotation.z = odom_data.pose.pose.orientation.z;

    // br.sendTransform(TransformStamped);//发布tf树
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vio_odom");
    ros::NodeHandle nh_;
    ROS_INFO("vio odom is start.......\n");
    vio_odom vio_odom_class(&nh_);

    ros::spin();
    return 0;

}