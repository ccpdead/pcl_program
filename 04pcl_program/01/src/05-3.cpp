#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxe_pass(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);          //?????????
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); //????????????
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);   //????????????

    // pcl::visualization::CloudViewer viewer("cloud viewer");
    pcl::visualization::PCLVisualizer viewer_pcl("cloud vierer pcl");
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ>("/home/ubuntu/test_pcd.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    //直通滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.6, 0.6);
    pass.filter(*cloud_pass);
    //直通滤波器
    pass.setInputCloud(cloud_pass);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.1, 1);
    pass.filter(*cloud_pass_2);

    cloud_filtered = cloud_pass_2;

    // //保存点云信息
    // pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZ> ("../test_pcd_downsampled.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.025);

    //////////////////////////////////////////////////////////////
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    //点云分割
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_p);
    ////////////////////////////////////////////////////
    seg.setInputCloud(cloud_p);
    seg.segment(*inliers, *coefficients);

    //点云分割
    extract.setInputCloud(cloud_p);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_p2);

    //半径滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem; //?????
    outrem.setInputCloud(cloud_p2);                  //??????
    outrem.setRadiusSearch(0.2);                     //?????0.8????????
    outrem.setMinNeighborsInRadius(50);              //?????????????2???
    // apply filter
    outrem.filter(*cloud_radius); //??????   ????0.8 ?????????????????????
                                  ////////////////////////////////////////

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extracter;
    feature_extracter.setInputCloud(cloud_radius);
    feature_extracter.compute();

    std::vector<float> momen_of_inertia;
    std::vector<float> eccentricity;

    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;

    Eigen::Matrix3f rotational_matrix_OBB; //旋转矩阵

    float major_value, middle_value, minor_value;

    Eigen::Vector3f major_vector, middle_vector, minor_vector; //存放
    Eigen::Vector3f mass_center;

    //?????
    feature_extracter.getMomentOfInertia(momen_of_inertia);

    //?????
    feature_extracter.getEccentricity(eccentricity);

    //??AABB??
    feature_extracter.getAABB(min_point_AABB, max_point_AABB);

    //??OBB??
    feature_extracter.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extracter.getEigenValues(major_value, middle_value, minor_value);

    //????major-ector? ??middle-vector? ???minor-vector
    feature_extracter.getEigenVectors(major_vector, middle_vector, minor_vector);

    //????
    feature_extracter.getMassCenter(mass_center);

    //////////////////////////////////////////
    viewer_pcl.setBackgroundColor(0.01, 0.01, 0.01);
    viewer_pcl.addCoordinateSystem(0.5);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 200, 0, 0);
    viewer_pcl.addPointCloud(cloud_filtered, single_color2, "cloud-vierer-pcl2");
    viewer_pcl.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud-vierer-pcl2");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer_pcl.addPointCloud(cloud_p, single_color, "cloud-vierer-pcl");
    viewer_pcl.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud-vierer-pcl");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud, 0, 0, 255);
    viewer_pcl.addPointCloud(cloud_p2, single_color3, "cloud-vierer-pcl3");
    viewer_pcl.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud-vierer-pcl3");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color4(cloud, 200, 200, 200);
    viewer_pcl.addPointCloud(cloud_radius, single_color4, "cloud-vierer-pcl4");
    viewer_pcl.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud-vierer-pcl4");

    //??AABB????
    viewer_pcl.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    viewer_pcl.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

    //??OBB????
    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    // Eigen::Quaternionf quat(rotational_matrix_OBB);

    Eigen::AngleAxisf rotation_vector(rotational_matrix_OBB); //旋转矩阵转化为旋转变量
    // Eigen::AngleAxisf rotation_vector_2(M_PI/4, Eigen::Vector3f(-0.5,0,0));
    Eigen::AngleAxisf rotation_vector_2(rotation_vector.angle(), Eigen::Vector3f(0, rotation_vector.axis().transpose().y(), rotation_vector.axis().transpose().z()));
    Eigen::Quaternionf quat(rotation_vector_2); //旋转矩阵转化为四元数

    // position 中心位置
    // quat 四元数
    // max_point_OBB.x - min_point_OBB.x 宽度
    // max_point_OBB.y - min_point_OBB.y 高度
    // max_point_OBB.z - min_point_OBB.z 深度

    viewer_pcl.addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer_pcl.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
    pcl::PointXYZ z_axiz(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));

    pcl::PointXYZ camera(0, 0, 0);
    viewer_pcl.addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer_pcl.addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer_pcl.addLine(center, z_axiz, 0.0f, 0.0f, 1.0f, "minor eigen vector");
    // viewer_pcl.addLine(center, camera, 1,1,1,"camera line");

    cout << "rotation_vector" << rotation_vector_2.angle() * (180 / M_PI) << endl;
    cout << "axis" << rotation_vector_2.axis().transpose() << endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> nest;
    nest.setInputCloud(cloud_radius);
    // normalEstimation.setRadiusSearch(0.1);
    nest.setKSearch(10);
    nest.compute(*normals);

    static int num = 1;
    float x = 0, y = 0, z = 0, nor_x = 0, nor_y = 0, nor_z = 0;
    for (size_t i = 0; i < cloud_radius->points.size(); ++i)
    { // ??????????????????????????????????
      // ??????????????pcl::concatenateFields

        if (normals->points[i].normal_x < 0.3 && normals->points[i].normal_x > 0)
        {
            if (normals->points[i].normal_z < -0.95 && normals->points[i].normal_z > -2.5)
            {

                num += 1;
                x += cloud_radius->points[i].x;
                y += cloud_radius->points[i].y;
                z += cloud_radius->points[i].z;

                nor_x += normals->points[i].normal_x;
                nor_y += normals->points[i].normal_y;
                nor_z += normals->points[i].normal_z;

                normals->points[i].x = cloud_radius->points[i].x;
                normals->points[i].y = cloud_radius->points[i].y;
                normals->points[i].z = cloud_radius->points[i].z;

                // cout<<x<<endl;
                // cout<<y<<endl;
                // cout<<z<<endl;
                // cout<<normals->points[i].normal_x<<endl;
                // cout<<normals->points[i].normal_y<<endl;
                // cout<<normals->points[i].normal_z<<endl;
                // cout<<endl;
            }
        }
    }

    // normals->points[1].x = (x/num);
    // normals->points[1].y = (y/num);
    // normals->points[1].z = (z/num);
    // normals->points[1].normal_x = (nor_x/num);
    // normals->points[1].normal_y = (nor_y/num);
    // normals->points[1].normal_z = (nor_z/num);
    // printf("%f,%f,%f,%f,%f,%f\r\n",(x/num),(y/num),(z/num),(nor_x/num),(nor_y/num),(nor_z/num));
    // printf("%f,%d\r\n",x,num);

    int level = 1;     // ?????????????
    float scale = 0.1; // ?????

    normals->points[0].x = center.x;
    // normals->points[0].y = center.y;
    normals->points[0].y = 0;
    normals->points[0].z = center.z;

    normals->points[0].normal_x = (nor_x / num);
    normals->points[0].normal_y = (nor_y / num);
    normals->points[0].normal_z = (nor_z / num);
    // printf("x %f,y %f,z %f\r\n",normals->points[0].normal_x ,normals->points[0].normal_y ,normals->points[0].normal_z);
    viewer_pcl.addPointCloudNormals<pcl::PointNormal>(normals, level, scale, "normalsing");

    viewer_pcl.spin();
    return 0;
}
