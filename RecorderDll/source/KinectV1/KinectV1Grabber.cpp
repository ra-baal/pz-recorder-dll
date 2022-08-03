#include "KinectV1Grabber.h"
#include "../additionals.h"

KinectV1Grabber::KinectV1Grabber() :
	_interface(new pcl::io::OpenNI2Grabber()),
	_viewerWasStopped(false)
{
	pcl::io::openni2::OpenNI2DeviceManager manager;
	_device = manager.getAnyDevice();

	if (_device == nullptr)
        throw DeviceNotFoundException("Nie znaleziono Kinecta V1.");

	_device->setSynchronization(true);
}

void KinectV1Grabber::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	if (!_viewerWasStopped)
	{
		cloudFromKinect = cloud;
		_viewerWasStopped = true;
	}
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr KinectV1Grabber::grabCloud()
{
	// ReSharper disable once CppUseFamiliarTemplateSyntaxForGenericLambdas
	const std::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = [this](auto&& PH1)
	{
		cloud_cb_(std::forward<decltype(PH1)>(PH1));
	};

	_interface->registerCallback(f);
	_interface->start();

	while (!_viewerWasStopped)
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	_interface->stop();

	return cloudFromKinect;
}
