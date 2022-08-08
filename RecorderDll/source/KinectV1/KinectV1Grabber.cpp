#include "KinectV1Grabber.h"
#include "../additionals.h"

//class NewColorFrameListener : openni::VideoStream::NewFrameListener
//{
//	// Inherited via NewFrameListener
//	virtual void onNewFrame(openni::VideoStream& viedoStream) override 
//	{
//		
//	}
//
//};


KinectV1Grabber::KinectV1Grabber() :
	_interface(new pcl::io::OpenNI2Grabber()),
	_viewerWasStopped(false)
{
	pcl::io::openni2::OpenNI2DeviceManager manager;
	_device = manager.getAnyDevice();
	
	if (_device == nullptr)
        throw DeviceNotFoundException("Nie znaleziono Kinecta V1.");

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

	LOG_IMPORTANT("_colorFrame.isValid: " << _colorFrame.isValid())

	_interface->stop();

	return cloudFromKinect;
}

ColorType* KinectV1Grabber::GetColorBufferData()
{
	std::clog << "KinectV1Grabber::GetColorBufferData()" << std::endl;
    return (ColorType*)_colorFrame.getData();
}

int KinectV1Grabber::GetColorWidth()
{
	std::clog << "KinectV1Grabber::GetColorWidth()" << std::endl;
    return _colorFrame.getWidth();
}

int KinectV1Grabber::GetColorHeight()
{
	std::clog << "KinectV1Grabber::GetColorHeight()" << std::endl;
    return _colorFrame.getHeight();
}
