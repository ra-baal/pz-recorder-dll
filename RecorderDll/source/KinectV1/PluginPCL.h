#pragma once

#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this

void flipEntity(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

// Link following functions C-style (required for plugins)
extern "C"
{
	class KinectCloudGrabber;

	EXPORT_API bool readCloud(LPCTSTR filename);
	EXPORT_API bool readKinectCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& output);
	EXPORT_API bool removeBiggestPlane(int maxIterations, double distanceThreshold);
	EXPORT_API bool getClusters(double clusterTolerance, int minClusterSize, int maxClusterSize);
	EXPORT_API int getClustersCount();
	EXPORT_API int getCloudSize();

	EXPORT_API bool getCluster(
		int clusterIndex, float** resultVertsX, float** resultVertsY, float** resultVertsZ,
		int** resultColorR, int** resultColorG, int** resultColorB,
		int* resultVertLength);

	EXPORT_API bool getClusterIndices(int clusterIndex, int** indices, int* indicesLength);

	EXPORT_API bool getCloud(
		float** resultVertsX, float** resultVertsY, float** resultVertsZ,
		int** resultColorR, int** resultColorG, int** resultColorB, int* resultVertLength);

	EXPORT_API bool readPointCloud(
			LPCTSTR filename, float** resultVertsX, float** resultVertsY, float** resultVertsZ,
			int** resultColorR, int** resultColorG, int** resultColorB, int* resultVertLength);

	EXPORT_API void freePointers(
		float** resultVertsX, float** resultVertsY, float** resultVertsZ,
		int** resultColorR, int** resultColorG, int** resultColorB);

	EXPORT_API void freeClusterIndices(int** indices);

}
