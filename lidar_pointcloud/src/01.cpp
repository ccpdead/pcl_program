#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<laser_geometry/laser_geometry.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/LaserScan.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>

class Costmap
{
    public:
        Costmap();
        ~Costmap()
        {

        }
        void camera_pointCallback(const sensor_msgs::PointCloud2::ConstPtr& point_msg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    private:
        ros::NodeHandle n;

        ros::Publisher cloud_pub;
        ros::Subscriber scan_sub;
        ros::Subscriber camera_sub;

        std::string subTopicName;
        std::string cloudTopicName;

        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        
        pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
        pcl::PointCloud<pcl::PointXYZ> cloud_camera;
};

Costmap::Costmap():n(ros::NodeHandle())
{
    ROS_INFO("start.............\r\n");
    scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &Costmap::scanCallback, this);//订阅雷达信息
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_pcl", 1000);//发布融合后的障碍物信息

    camera_sub = n.subscribe<sensor_msgs::PointCloud2>("/rgbd_camera_pointcloud/proj_obstacles", 1000, &Costmap::camera_pointCallback, this);//订阅深度相机octomap沉降后的障碍物信息
}
//接受雷达信息并转化为base——footprint坐标系下的点云信息，并转化为PCL的点云信息。
void Costmap::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    sensor_msgs::PointCloud2 cloud;
    if(!tfListener_.waitForTransform(
        scan_msg->header.frame_id,
        "base_link",
        scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size() * scan_msg->time_increment),
        ros::Duration(2)))
        {
            return;
        }
    projector_.transformLaserScanToPointCloud("base_link", *scan_msg, cloud, tfListener_);
    pcl::fromROSMsg(cloud, this->cloud_lidar);//将ROS数据转化为点云数据
    // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    
    output_cloud = this->cloud_camera + this->cloud_lidar;//将雷达点云信息与深度点云信息结合
    sensor_msgs::PointCloud2 cloud_ros_output;

    pcl::toROSMsg(output_cloud, cloud_ros_output);//将点云数据转化为ROS格式
    cloud_pub.publish(cloud_ros_output);//发布ROS点云数据

}

//接受相机的点云信息，并转化为PCL的类型。
void Costmap::camera_pointCallback(const sensor_msgs::PointCloud2::ConstPtr& point_msg)
{
    pcl::fromROSMsg(*point_msg, this->cloud_camera);
}

int main(int argc, char** argv){

    ros::init(argc, argv,"lidar_point");
    Costmap costmap;
    ros::spin();
    return 0;
}