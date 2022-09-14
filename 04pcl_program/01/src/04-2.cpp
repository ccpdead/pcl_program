#include <iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

#include<pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_visualizer.h>

/**
 * @brief 显示pcd点云信息数据
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int
main(int argc, char** argv)
{
    pcl::visualization::CloudViewer viewer_pcl("cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_viewer (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader file_read;
    file_read.read<pcl::PointXYZ>("/home/ubuntu/test_pcd.pcd",*cloud_viewer);

    std::cerr<<*cloud_viewer<<std::endl;
    viewer_pcl.showCloud(cloud_viewer);
    while(!viewer_pcl.wasStopped())
    {

    }
    return 0;


}