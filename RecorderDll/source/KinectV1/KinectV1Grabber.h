/// Zmodyfikowana klasa KinectCloudGrabber z PluginPCL.cpp

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_device_manager.h>

class KinectV1Grabber
{
private:
	pcl::Grabber* _interface;
	bool _viewerWasStopped;
	pcl::io::openni2::OpenNI2Device::Ptr _device;

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);

public:
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudFromKinect;

	KinectV1Grabber();
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr grabCloud();
};
