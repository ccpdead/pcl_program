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

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);
	//暂存一个面的点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);

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

	// 将直通滤波后的数据保存
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
	seg.setDistanceThreshold(0.03); // 0.025
	//////////////////////////////////////////////////////////////
	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);
	//点云分隔
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_p);
	////////////////////////////////////////////////////
	seg.setInputCloud(cloud_p);
	seg.segment(*inliers, *coefficients);
	//点云分隔
	extract.setInputCloud(cloud_p);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_p2);

	//半径滤波器
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud_p2);
	outrem.setRadiusSearch(0.18);
	outrem.setMinNeighborsInRadius(50);
	// apply filter
	outrem.filter(*cloud_radius);
	////////////////////////////////////////
	//检测法线
	pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> nest;
	nest.setInputCloud(cloud_radius);
	// normalEstimation.setRadiusSearch(0.1);
	nest.setKSearch(10);
	nest.compute(*normals);

	static int num = 0;
	float x = 0, y = 0, z = 0, nor_x = 0, nor_y = 0, nor_z = 0;
	for (size_t i = 0; i < cloud_radius->points.size(); ++i)
	{
		if (normals->points[i].normal_x < 0.3 && normals->points[i].normal_x > -0.2)
		{
			if (normals->points[i].normal_z < -0.95 && normals->points[i].normal_z > -2.5)
			{
				//将符合条件的点云保存
				cloud_final->insert(cloud_final->end(), cloud_radius->points[i]);
			}
		}
	}

	//创建惯性矩阵估计对象,设置输入点云,进行运算
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extracter;
	feature_extracter.setInputCloud(cloud_final);
	feature_extracter.compute();

	std::vector<float> momen_of_inertia;
	std::vector<float> eccentricity;
	pcl::PointXYZ min_point_OBB; //存放OBB盒子尺寸信息
	pcl::PointXYZ max_point_OBB;

	pcl::PointXYZ position_OBB;			   //存放OBB盒子中心位置信息
	Eigen::Matrix3f rotational_matrix_OBB; //旋转矩阵
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center; //存放质心

	//获得惯性矩
	feature_extracter.getMomentOfInertia(momen_of_inertia);
	//获取离心率
	feature_extracter.getEccentricity(eccentricity);
	//获取OBB盒子
	feature_extracter.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	//获取主轴major-ector， 中轴middle-vector， 辅助轴minor-vector
	feature_extracter.getEigenValues(major_value, middle_value, minor_value);
	//获取主轴major-ector， 中轴middle-vector， 辅助轴minor-vector --->注意,这些都是向量类型
	feature_extracter.getEigenVectors(major_vector, middle_vector, minor_vector);
	//获取质心
	feature_extracter.getMassCenter(mass_center);

	//---------------------------------------------------------------------------------
	//添加OBB包容盒子
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	// Eigen::Quaternionf quat(rotational_matrix_OBB);
	Eigen::Vector3f euler_angles = rotational_matrix_OBB.eulerAngles(2, 1, 0); //旋转矩阵转化为欧拉角
	cout << "euler angle x " << euler_angles.x() * (180 / M_PI) << endl;
	cout << "euler angle y " << euler_angles.y() * (180 / M_PI) << endl;
	cout << "euler angle z " << euler_angles.z() * (180 / M_PI) << endl;

	if (euler_angles.z() > 1.57 || euler_angles.z() < -1.57)
	{
		euler_angles.y() = 3.14 - euler_angles.y();
	}
	//------------------------------------------
	if (euler_angles.y() > 1.57)
	{
		euler_angles.y() = euler_angles.y() - 3.14;
	}
	else if (euler_angles.y() < -1.57)
	{
		euler_angles.y() = 3.14 + euler_angles.y();
	}

	Eigen::Vector3f euler_angles2(0, euler_angles.y(), 0);
	//欧拉角转换为四元数
	Eigen::Quaternionf quat = Eigen::AngleAxisf(euler_angles2[0], Eigen::Vector3f::UnitZ()) *
							  Eigen::AngleAxisf(euler_angles2[1], Eigen::Vector3f::UnitY()) *
							  Eigen::AngleAxisf(euler_angles2[2], Eigen::Vector3f::UnitX());

	cout << "----------------------------------" << endl;
	cout << "position x " << position.x() << endl;
	cout << "position y " << 0 << endl;
	cout << "position z " << position.z() << endl;
	cout << endl;
	cout << "euler angle x " << euler_angles2.x() * (180 / M_PI) << endl;
	cout << "euler angle y " << euler_angles2.y() * (180 / M_PI) << endl;
	cout << "euler angle z " << euler_angles2.z() * (180 / M_PI) << endl;
	cout << "---------------------------------" << endl;
	// position：中心位置
	// quat：旋转矩阵
	// max_point_OBB.x - min_point_OBB.x 宽度
	// max_point_OBB.y - min_point_OBB.y 高度
	// max_point_OBB.z - min_point_OBB.z 深度
	viewer_pcl.addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	viewer_pcl.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

	pcl::PointXYZ camera(0, 0, 0);
	pcl::PointXYZ centor(position.x(), 0, position.z());

	Eigen::Affine3f transformtion_matrix;							 //定义仿射变换
	Eigen::Translation3f translation(position.x(), 0, position.z()); //定义平移
	transformtion_matrix = translation * quat.toRotationMatrix();	 //变量初始化

	viewer_pcl.addLine(centor, camera, 1, 1, 1, "camera line");
	// viewer_pcl.addCoordinateSystem(0.5,position.x(),position.y(),position.z());
	viewer_pcl.addCoordinateSystem(0.5, transformtion_matrix);
	//----------------------------------------------------------------------------

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

	//显示最终的点云数据
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color5(cloud, 0, 200, 200);
	viewer_pcl.addPointCloud(cloud_final, single_color5, "cloud-vierer-pcl5");
	viewer_pcl.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud-vierer-pcl5");
	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	viewer_pcl.spin();
	return 0;
}
