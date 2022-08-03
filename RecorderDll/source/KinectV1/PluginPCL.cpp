#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/boost.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>
#include <pcl/io/openni2/openni2_device_manager.h>
#include <pcl/common/transforms.h>

#include "PluginPCL.h"


class KinectCloudGrabber
{
	pcl::Grabber* interface;
	bool viewerWasStopped;

public:
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudFromKinect;

	KinectCloudGrabber() 
	: interface(new pcl::io::OpenNI2Grabber()), viewerWasStopped(false)
	{
	}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
	{
		if (!viewerWasStopped)
		{
			cloudFromKinect = cloud;
			viewerWasStopped = true;
		}
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr grabCloud()
	{
		pcl::io::openni2::OpenNI2DeviceManager manager;
		const pcl::io::openni2::OpenNI2Device::Ptr device = manager.getAnyDevice();
		device->setSynchronization(true);

		// ReSharper disable once CppUseFamiliarTemplateSyntaxForGenericLambdas
		const std::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = [this](auto&& PH1)
		{
			cloud_cb_(std::forward<decltype(PH1)>(PH1));
		};

		interface->registerCallback(f);
		interface->start();

		while (!viewerWasStopped)
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		interface->stop();

		return cloudFromKinect;
	}
};

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr maincloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr* clusteredclouds;
std::vector<std::vector<int>> clusteredcloudsindices;

int clustersCount = 0;

/**
 * \brief Flips the passed entity on the Z-Axis
 * \param cloud The entity to be flipped
 */
void flipEntity(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.rotate(Eigen::AngleAxisf(22 / 7.0, Eigen::Vector3f::UnitZ()));
	transformPointCloud(*cloud, *cloud, transform_2);
}

bool readCloud(LPCTSTR filename)
{
	pcl::PCDReader reader;
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	reader.read(filename, *cloud);

	if (cloud->empty())
	{
		return false;
	}

	flipEntity(cloud);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*maincloud);

	return true;
}

bool readKinectCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& output)
{
	KinectCloudGrabber kinectCloudGrabber;
	const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud = kinectCloudGrabber.grabCloud();
	// std::cout << "Cloudsize :" << cloud->points.size () <<  std::endl;

	if (cloud->empty())
	{
		return false;
	}
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*output);

	return true;
}

bool removeBiggestPlane(int maxIterations, double distanceThreshold)
{
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	const pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	const pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIterations); // 100);
	seg.setDistanceThreshold(distanceThreshold); // 0.02);

	int i = 0;
	const int nr_points = static_cast<int>(maincloud->points.size());
	while (maincloud->points.size() > (0.3) * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(maincloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.empty())
		{
			// std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud(maincloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		// std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*maincloud = *cloud_f;
	}

	return true;
}

bool getClusters(double clusterTolerance, int minClusterSize, int maxClusterSize)
{
	// Creating the KdTree object for the search method of the extraction
	const pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(maincloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(clusterTolerance); // 0.02); // 2cm
	ec.setMinClusterSize(minClusterSize); // 100);
	ec.setMaxClusterSize(maxClusterSize); // 25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(maincloud);
	ec.extract(cluster_indices);

	int j = 0;

	clustersCount = cluster_indices.size();
	clusteredclouds = new pcl::PointCloud<pcl::PointXYZRGBA>::Ptr[clustersCount];
	clusteredcloudsindices.resize(clustersCount);

	for (const auto& cluster_indice : cluster_indices)
	{
		const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);

		for (int indice : cluster_indice.indices)
		{
			cloud_cluster->points.push_back(maincloud->points[indice]); //*
			clusteredcloudsindices[j].push_back(indice);
		}

		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		clusteredclouds[j] = cloud_cluster;

		j++;
	}

	return true;
}

int getClustersCount()
{
	return clustersCount;
}

int getCloudSize()
{
	return maincloud->points.size();
}

bool getCluster(
	int clusterIndex, float** resultVertsX, float** resultVertsY, float** resultVertsZ,
	int** resultColorR, int** resultColorG, int** resultColorB,
	int* resultVertLength)
{
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster = clusteredclouds[clusterIndex];

	if (cluster->empty())
	{
		return false;
	}

	const auto rVertsX = new float[cluster->points.size()];
	const auto rVertsY = new float[cluster->points.size()];
	const auto rVertsZ = new float[cluster->points.size()];
	const auto rColorR = new int[cluster->points.size()];
	const auto rColorG = new int[cluster->points.size()];
	const auto rColorB = new int[cluster->points.size()];

	for (size_t i = 0; i < cluster->points.size(); ++i)
	{
		rVertsX[i] = (*cluster)[i].x;
		rVertsY[i] = (*cluster)[i].y;
		rVertsZ[i] = (*cluster)[i].z;
		rColorR[i] = (*cluster)[i].r;
		rColorG[i] = (*cluster)[i].g;
		rColorB[i] = (*cluster)[i].b;
	}

	*resultVertsX = rVertsX;
	*resultVertsY = rVertsY;
	*resultVertsZ = rVertsZ;
	*resultColorR = rColorR;
	*resultColorG = rColorG;
	*resultColorB = rColorB;

	const int rVertLength = cluster->points.size();
	*resultVertLength = rVertLength;

	return true;
}

bool getClusterIndices(int clusterIndex, int** indices, int* indicesLength)
{
	const std::vector<int> clusteredcloudindices = clusteredcloudsindices[clusterIndex];

	if (clusteredcloudindices.empty())
	{
		return false;
	}

	const auto inds = new int[clusteredcloudindices.size()];

	for (size_t i = 0; i < clusteredcloudindices.size(); ++i)
	{
		inds[i] = clusteredcloudindices[i];
	}

	*indices = inds;

	//std::ofstream file;
	//file.open("logCluster.txt", std::ios::trunc);
	//file << "Pointers after allocation" << std::endl;
	//file << std::hex << *indices << std::endl << std::endl;
	//file.close();

	const int indLength = clusteredcloudindices.size();
	*indicesLength = indLength;

	return true;
}

bool getCloud(
	float** resultVertsX, float** resultVertsY, float** resultVertsZ,
	int** resultColorR, int** resultColorG, int** resultColorB, int* resultVertLength)
{
	if (maincloud->empty())
	{
		return false;
	}

	const auto rVertsX = new float[maincloud->points.size()];
	const auto rVertsY = new float[maincloud->points.size()];
	const auto rVertsZ = new float[maincloud->points.size()];
	const auto rColorR = new int[maincloud->points.size()];
	const auto rColorG = new int[maincloud->points.size()];
	const auto rColorB = new int[maincloud->points.size()];

	for (size_t i = 0; i < maincloud->points.size(); ++i)
	{
		rVertsX[i] = (*maincloud)[i].x;
		rVertsY[i] = (*maincloud)[i].y;
		rVertsZ[i] = (*maincloud)[i].z;
		rColorR[i] = (*maincloud)[i].r;
		rColorG[i] = (*maincloud)[i].g;
		rColorB[i] = (*maincloud)[i].b;
	}

	*resultVertsX = rVertsX;
	*resultVertsY = rVertsY;
	*resultVertsZ = rVertsZ;
	*resultColorR = rColorR;
	*resultColorG = rColorG;
	*resultColorB = rColorB;

	//std::ofstream file;
	//file.open("log.txt", std::ios::trunc);
	//file << "Pointers after allocation" << std::endl;
	//file << std::hex << *resultVertsX << std::endl;
	//file << std::hex << *resultVertsY << std::endl;
	//file << std::hex << *resultVertsZ << std::endl;
	//file << std::hex << *resultColorR << std::endl;
	//file << std::hex << *resultColorG << std::endl;
	//file << std::hex << *resultColorB << std::endl << std::endl;
	//file.close();

	const int rVertLength = maincloud->points.size();
	*resultVertLength = rVertLength;

	return true;
}

bool readPointCloud(LPCTSTR filename, float** resultVertsX, float** resultVertsY, float** resultVertsZ,
	int** resultColorR, int** resultColorG, int** resultColorB, int* resultVertLength)
{
	// Read in the cloud data
	pcl::PCDReader reader;
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	reader.read(filename, *cloud);

	if (cloud->empty())
	{
		return false;
	}

	const auto rVertsX = new float[cloud->points.size()];
	const auto rVertsY = new float[cloud->points.size()];
	const auto rVertsZ = new float[cloud->points.size()];
	const auto rColorR = new int[cloud->points.size()];
	const auto rColorG = new int[cloud->points.size()];
	const auto rColorB = new int[cloud->points.size()];

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		rVertsX[i] = (*cloud)[i].x;
		rVertsY[i] = (*cloud)[i].y;
		rVertsZ[i] = (*cloud)[i].z;
		rColorR[i] = (*cloud)[i].r;
		rColorG[i] = (*cloud)[i].g;
		rColorB[i] = (*cloud)[i].b;
	}

	*resultVertsX = rVertsX;
	*resultVertsY = rVertsY;
	*resultVertsZ = rVertsZ;
	*resultColorR = rColorR;
	*resultColorG = rColorG;
	*resultColorB = rColorB;

	const int rVertLength = cloud->points.size();
	*resultVertLength = rVertLength;

	return true;
}

void freePointers(
	float** resultVertsX, float** resultVertsY, float** resultVertsZ,
	int** resultColorR, int** resultColorG, int** resultColorB)
{
	//std::ofstream file;
	//file.open("log.txt", std::ios::app);
	//file << "Pointers before deallocation" << std::endl;
	//file << std::hex << resultVertsX << std::endl;
	//file << std::hex << resultVertsY << std::endl;
	//file << std::hex << resultVertsZ << std::endl;
	//file << std::hex << resultColorR << std::endl;
	//file << std::hex << resultColorG << std::endl;
	//file << std::hex << resultColorB << std::endl;
	//file.close();

	delete[] resultVertsX;
	delete[] resultVertsY;
	delete[] resultVertsZ;
	delete[] resultColorR;
	delete[] resultColorG;
	delete[] resultColorB;
}

void freeClusterIndices(int** indices)
{
	//std::ofstream file;
	//file.open("logCluster.txt", std::ios::app);
	//file << "Pointers before deallocation" << std::endl;
	//file << std::hex << indices << std::endl << std::endl;
	//file.close();

	delete[] indices;
}

/*
bool grabPointCloudFromKinect(float** resultVertsX, float** resultVertsY, float** resultVertsZ, int* resultVertLength)
{

		KinectCloudGrabber kinectCloudGrabber;
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud = kinectCloudGrabber.grabCloud ();
		std::cout << "Cloudsize :" << cloud->points.size () <<  std::endl;

	if (cloud->empty()) {
		return false;
	}

	float* rVertsX = new float[cloud->points.size ()];
	float* rVertsY = new float[cloud->points.size ()];
	float* rVertsZ = new float[cloud->points.size ()];

	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		rVertsX[i] = cloud->points[i].x;
		rVertsY[i] = cloud->points[i].y;
		rVertsZ[i] = cloud->points[i].z;
	}

	*resultVertsX = rVertsX;
	*resultVertsY = rVertsY;
	*resultVertsZ = rVertsZ;

	int rVertLength = cloud->points.size ();
	*resultVertLength = rVertLength;

	return true;
}
*/

/// There is not needed main function in dll library?
void kinect_v1_main() 
{	
	//getchar();

	//std::cout << "Reading" << std::endl;
	//std::cout << readCloud("test.pcd") << std::endl;
	try
	{
		// Tworzenie tego obiektu urz¹dzenia tutaj jest chyba niepotrzebne,
		// bo dok³adnie to samo potem dzieje siê w KinectCloudGrabber::grabCloud().
		pcl::io::openni2::OpenNI2DeviceManager manager;
		pcl::io::openni2::OpenNI2Device::Ptr device = manager.getAnyDevice();

		std::cout << "Device name: " << device->getName() << std::endl;

		device->setSynchronization(true);
		std::cout << "Synchronization set." << std::endl;
		readKinectCloud(maincloud);
		std::cout << "Cloud read." << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}

	std::cout << "0" << std::endl;
	//removeBiggestPlane(100, 0.02);
	std::cout << "1" << std::endl;
	getClusters(0.02, 100, 25000);
	std::cout << "2" << std::endl;
	getClustersCount();
	std::cout << "3" << std::endl;

	//for (const int i : clusteredcloudsindices[0])
	//{
	//	std::cout << " : " << i;
	//}

	//std::cout << std::endl;

	//std::cout << "0 -> " << clusteredclouds[0]->size() << std::endl;
	//std::cout << "1 -> " << clusteredclouds[1]->size() << std::endl;
	//std::cout << "2 -> " << clusteredclouds[2]->size() << std::endl;
	//std::cout << "3 -> " << clusteredclouds[3]->size() << std::endl;
	//std::cout << "4 -> " << clusteredclouds[4]->size() << std::endl;

	std::cout << "KinectV1: end" << std::endl;

}


