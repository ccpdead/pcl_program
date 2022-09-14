#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>

/**
 * @brief 订阅点云信息，并保存至本地pcd文件
 * 
 * @param input 
 */
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    printf("got the pcl data--------\r\n");

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg = *input;
    pcl::PointCloud<pcl::PointXYZ>cloud_pcl;
    pcl::fromROSMsg(cloud_msg, cloud_pcl);
    pcl::io::savePCDFile("/home/ubuntu/test_pcd.pcd",cloud_pcl);
    std::cerr << cloud_pcl << std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"pcl_save_pcd");
    ros::NodeHandle nh_;
    printf("start received the pointlcoud\r\n");
    //订阅深度点云信息
    // ros::Subscriber sub = nh_.subscribe("/filled_depth_pointcloud/depth_pointcloud", 1,cloud_cb);
    ros::Subscriber sub = nh_.subscribe("/sunny_topic/tof_frame/pointcloud", 1,cloud_cb);

    char input;
    while(ros::ok())
    {
        std::cin>>input;
        if(input == 's')
        {
            ros::spinOnce();
        }else if(input == 'q')
        {
            break;
        }
    }
    return 0;
}