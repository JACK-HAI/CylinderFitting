#include <pcl/ModelCoefficients.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "_3dLineDistance.h"

typedef pcl::PointXYZ PointT;
#define DistanceThreshold 0.09
#define LeafSize 0.1f

int main(int argc, char** argv)
{
	// All the objects needed
	pcl::OBJReader reader;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_voxelFiltered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	pcl::PointCloud<pcl::Normal>::Ptr normals1_complementarySet(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_cylinder1(new pcl::ModelCoefficients), coefficients_cylinder2(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder1(new pcl::PointIndices), inliers_cylinder2(new pcl::PointIndices);

	// Read in the cloud data
	reader.read("D:\\文件\\哈特工作\\管子检测\\20170906_连续直段\\1.obj", *cloud);
//	reader.read("4.obj", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	//体素向下采样
	// Create the filtering object
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(LeafSize, LeafSize, LeafSize);
	vg.filter(*cloud_voxelFiltered);
	std::cerr << "PointCloud after voxelGrided has: " << cloud_voxelFiltered->points.size() << " data points." << std::endl;


	*cloud_voxelFiltered = *cloud;
	//Create the StatisticalOutlier object
	pcl::PointCloud<PointT>::Ptr cloud_StatisticalOutlierRemoved(new pcl::PointCloud<PointT>);
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud_voxelFiltered);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1);
	sor.filter(*cloud_StatisticalOutlierRemoved);

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_StatisticalOutlierRemoved);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the cylinder1 model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setRadiusLimits(1.5, 3);
	seg.setMaxIterations(10000);
	seg.setProbability(0.99999);
	seg.setDistanceThreshold(DistanceThreshold);
	seg.setInputCloud(cloud_StatisticalOutlierRemoved);
	seg.setInputNormals(cloud_normals);
	// Obtain the cylinder1 inliers and coefficients
	seg.segment(*inliers_cylinder1, *coefficients_cylinder1);
	std::cerr << "cylinder1 coefficients: " << *coefficients_cylinder1 << std::endl;

	// Extract the cylinder1 inliers from the input cloud
	extract.setInputCloud(cloud_StatisticalOutlierRemoved);
	extract.setIndices(inliers_cylinder1);
	extract.setNegative(false);

	// Write the cylinder1 inliers to disk
	pcl::PointCloud<PointT>::Ptr cloud_cylinder1(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder1);
	std::cerr << "PointCloud representing the planar component: " << cloud_cylinder1->points.size() << " data points." << std::endl;

	// Remove the cylinder1 inliers, extract the rest
	extract.setNegative(true);
	pcl::PointCloud<PointT>::Ptr cylinder1_ComplementarySet(new pcl::PointCloud<PointT>());
	extract.filter(*cylinder1_ComplementarySet);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_cylinder1);
	extract_normals.filter(*normals1_complementarySet);

	// Create the segmentation object for cylinder2 segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(100000);
	seg.setProbability(0.999999);
	seg.setDistanceThreshold(DistanceThreshold);
//	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(cylinder1_ComplementarySet);
	seg.setInputNormals(normals1_complementarySet);


	
	// Obtain the cylinder2 inliers and coefficients
	seg.segment(*inliers_cylinder2, *coefficients_cylinder2);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder2 << std::endl;

	// Write the cylinder2 inliers to disk
	extract.setInputCloud(cylinder1_ComplementarySet);
	extract.setIndices(inliers_cylinder2);
	extract.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder2(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder2);

	//保存 cylinder2 数据到 pcd 文件
	pcl::PCDWriter pcdw;
	pcdw.writeASCII("points.pcd", *cloud_cylinder2);

	// Remove the cylinder2 inliers, extract the rest
	extract.setNegative(true);
	pcl::PointCloud<PointT>::Ptr cylinder2_ComplementarySet(new pcl::PointCloud<PointT>());
	extract.filter(*cylinder2_ComplementarySet);


	Eigen::Matrix3f translatedCoordinate;
	Eigen::Vector3f line1_point(coefficients_cylinder1->values[0], coefficients_cylinder1->values[1], coefficients_cylinder1->values[2]);
	Eigen::Vector3f line1_vector(coefficients_cylinder1->values[3], coefficients_cylinder1->values[4], coefficients_cylinder1->values[5]);
	Eigen::Vector3f line2_point(coefficients_cylinder2->values[0], coefficients_cylinder2->values[1], coefficients_cylinder2->values[2]);
	Eigen::Vector3f line2_vector(coefficients_cylinder2->values[3], coefficients_cylinder2->values[4], coefficients_cylinder2->values[5]);
	caculateLineDistance3D line_distance(line1_point, line1_vector, line2_point, line2_vector);
	float linedistance = line_distance.getLineDistance();
	cout << "linedistance = " << linedistance << endl;
	std::vector<Eigen::Vector3f> vec_TranslatedPoints = line_distance.getTranslatedPoints();
	std::vector<Eigen::Vector3f> vec_LinePoints = line_distance.getLinePoints();
	Eigen::Vector3f crossPoint = line_distance.getCrossPoints();
	cout << "crossPoint = " << crossPoint << endl;
		//显示
	//	pcl::visualization::CloudViewer viewer("Cloud Sphere");
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud cylinder"));
		viewer->initCameraParameters();
		viewer->setSize(1920, 1080);

		viewer->setBackgroundColor(0, 0, 0);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_white(cylinder2_ComplementarySet, 255, 255, 255);
		viewer->addPointCloud<pcl::PointXYZ>(cylinder2_ComplementarySet, color_handler_white, "Cloud cylinder");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_green(cloud_cylinder1, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder1, color_handler_green, "Cloud cylinder1");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Cloud cylinder1");
		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_red(cloud_cylinder2, 255, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder2, color_handler_red, "Cloud cylinder2");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud cylinder2");

		while (!viewer->wasStopped())
		{
			//you can also do cool processing here
			//FIXME: Note that this is running in a separate thread from viewerPsycho
			//and you should guard against race conditions yourself...
			viewer->spinOnce(100);
		}

	return (0);
}