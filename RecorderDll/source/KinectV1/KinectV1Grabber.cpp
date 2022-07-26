#include "KinectV1Grabber.h"
#include "../additionals.h"

KinectV1Grabber::KinectV1Grabber() :
	_interface(new pcl::io::OpenNI2Grabber()),
	_viewerWasStopped(false)
{
	pcl::io::openni2::OpenNI2DeviceManager manager;

	_device = manager.getAnyDevice();
	
	if (_device == nullptr)
        throw DeviceNotFoundException("Kinect V1 not found");

	_device->setSynchronization(true);


	_setStreams();

}

void KinectV1Grabber::_setStreams()
{
	 pcl::io::openni2::OpenNI2Device::StreamCallbackFunction colorCallback = 
	 [this](openni::VideoStream& stream)
	 {
		stream.readFrame(&_colorFrame);
	 };

	_device->setColorCallback(colorCallback);

	_device->startColorStream();
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
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	//LOG("_colorFrame.isValid: " << _colorFrame.isValid())
	//auto mode = _colorFrame.getVideoMode();
	//auto format = mode.getPixelFormat();
	//LOG(format);
	//LOG(_colorFrame.getDataSize());
	//LOG(_colorFrame.getHeight());
	//LOG(_colorFrame.getWidth());

	_interface->stop();

	return cloudFromKinect;
}

Rgb24* KinectV1Grabber::GetColorBufferData()
{
	//std::clog << "KinectV1Grabber::GetColorBufferData()" << std::endl;
    return (Rgb24*)_colorFrame.getData();
}

int KinectV1Grabber::GetColorWidth()
{
	//std::clog << "KinectV1Grabber::GetColorWidth()" << std::endl;
    return _colorFrame.getWidth();
}

int KinectV1Grabber::GetColorHeight()
{
	//std::clog << "KinectV1Grabber::GetColorHeight()" << std::endl;
    return _colorFrame.getHeight();
}
