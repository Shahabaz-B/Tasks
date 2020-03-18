#define _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>



inline void ViewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void ExtractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, pcl::ModelCoefficients::Ptr coefficients, double distranceThresh = 2);

Eigen::Matrix4f AlignTo_XY_Plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_points, pcl::ModelCoefficients::Ptr coefficients);

void FindNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

void DoPCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected);

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::cout << "enter file path\n";
	std::string filepath;
	std::cin >> filepath;
	pcl::PLYReader Reader;
	if (Reader.read(filepath, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud((new pcl::PointCloud<pcl::PointXYZRGB>));
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	ExtractPlane(cloud, planeCloud, coefficients);
	
	ViewCloud(planeCloud);

	Eigen::Matrix4f matrix;
	matrix = AlignTo_XY_Plane(planeCloud, coefficients);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr centeredCloud((new pcl::PointCloud<pcl::PointXYZRGB>));
	pcl::transformPointCloud(*cloud, *centeredCloud, matrix);

	Eigen::Vector4f min, max;
	pcl::getMinMax3D(*centeredCloud, min, max);

	std::cout << "Max dimension on each axis : " << max;

	ViewCloud(centeredCloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	DoPCA(cloud, cloudPointsProjected);

	ViewCloud(cloudPointsProjected);

	return (0);
}

void FindNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(input);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.03);

	// Compute the features
	ne.compute(*cloud_normals);
}

void ExtractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, pcl::ModelCoefficients::Ptr coefficients, double distranceThresh)
{
	
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(7);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud(*cloud, *inliers, *output);
	
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);

	// Get the points associated with the planar surface
	extract.filter(*output);
	std::cout << "PointCloud representing the planar component: " << output->points.size() << " data points." << std::endl;

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud);
}

inline void ViewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
	viewer->addCoordinateSystem(1000.0);
	viewer->initCameraParameters();
	viewer->spin();
}

void DoPCA (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected)
{
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
	
	pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
}

Eigen::Matrix4f AlignTo_XY_Plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_points, pcl::ModelCoefficients::Ptr coefficients)
{

	Eigen::Vector3f floor_plane_normal_vector, xy_plane_normal_vector;

	
	// Setting the floor normal plane as coefficients of plane
	floor_plane_normal_vector[0] = coefficients->values[0];
	floor_plane_normal_vector[1] = coefficients->values[1];
	floor_plane_normal_vector[2] = coefficients->values[2];

	std::cout << floor_plane_normal_vector << std::endl;

	// coefficients of normal to XY plane
	xy_plane_normal_vector[0] = 0.0;
	xy_plane_normal_vector[1] = 0.0;
	xy_plane_normal_vector[2] = 1.0;

	std::cout << xy_plane_normal_vector << std::endl;

	Eigen::Vector3f rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
	float theta = acos(coefficients->values[2] / sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) + pow(coefficients->values[2], 2)));

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	transform_1(0, 0) = pow(rotation_vector[0], 2) * (1 - cos(theta)) + cos(theta);
	transform_1(0, 1) = (rotation_vector[1] * rotation_vector[0]) * (1 - cos(theta)) - rotation_vector[2] * sin(theta);
	transform_1(0, 2) = (rotation_vector[2] * rotation_vector[0]) * (1 - cos(theta)) + rotation_vector[1] * sin(theta);
	transform_1(0, 3) = 3.0;


	transform_1(1, 0) = (rotation_vector[0] * rotation_vector[1]) * (1 - cos(theta)) + rotation_vector[2] * sin(theta);
	transform_1(1, 1) = pow(rotation_vector[1], 2) * (1 - cos(theta)) + cos(theta);
	transform_1(1, 2) = (rotation_vector[2] * rotation_vector[1]) * (1 - cos(theta)) - rotation_vector[0] * sin(theta);

	transform_1(2, 0) = (rotation_vector[0] * rotation_vector[2]) * (1 - cos(theta)) - rotation_vector[1] * sin(theta);
	transform_1(2, 1) = (rotation_vector[1] * rotation_vector[2]) * (1 - cos(theta)) + rotation_vector[0] * sin(theta);
	transform_1(2, 2) = pow(rotation_vector[2], 2) * (1 - cos(theta)) + cos(theta);

	return transform_1;
}

void GetMinMax3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointXYZRGB &min, pcl::PointXYZRGB &max)
{
	min.x = (*min_element(points->begin(), points->end(), [](pcl::PointXYZRGB &lhs, pcl::PointXYZRGB & rhs) { return lhs.x < rhs.x; })).x;
	max.x = (*max_element(points->begin(), points->end(), [](pcl::PointXYZRGB &lhs, pcl::PointXYZRGB & rhs) { return lhs.x < rhs.x; })).x;
	min.y = (*min_element(points->begin(), points->end(), [](pcl::PointXYZRGB &lhs, pcl::PointXYZRGB & rhs) { return lhs.y < rhs.y; })).y;
	max.y = (*max_element(points->begin(), points->end(), [](pcl::PointXYZRGB &lhs, pcl::PointXYZRGB & rhs) { return lhs.y < rhs.y; })).y;
	min.z = (*min_element(points->begin(), points->end(), [](pcl::PointXYZRGB &lhs, pcl::PointXYZRGB & rhs) { return lhs.z < rhs.z; })).z;
	max.z = (*max_element(points->begin(), points->end(), [](pcl::PointXYZRGB &lhs, pcl::PointXYZRGB & rhs) { return lhs.z < rhs.z; })).z;
}