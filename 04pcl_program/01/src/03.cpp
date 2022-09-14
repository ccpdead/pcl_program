#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include<pcl/filters/voxel_grid.h>
/**
 * @brief 接收点云信息并通过体素滤波处理后重新发送
 * 
 */
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;//定义指针
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);//定义指针
    pcl::PCLPointCloud2 cloud_filtered;

    pcl_conversions::toPCL(*input, *cloud);//将ROS信息转换为PCL信息。

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;//体素滤波
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.03, 0.03, 0.03);
    sor.filter(cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    pub.publish(output);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "pcl_tutorial_01");
    ros::NodeHandle nh_;

    //订阅深度点云信息。
    ros::Subscriber sub = nh_.subscribe("camera/depth/color/points", 1, cloud_cb);

    pub = nh_.advertise<sensor_msgs::PointCloud2>("output", 1);

    ros::spin();

}