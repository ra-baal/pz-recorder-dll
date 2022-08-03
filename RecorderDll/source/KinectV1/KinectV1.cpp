#include "KinectV1.h"

#include "../additionals.h"

KinectV1::KinectV1() 
{
	std::clog << "KinectV1::KinectV1()" << std::endl;
    _kinectV1Grabber = std::make_shared<KinectV1Grabber>();
}

KinectV1::~KinectV1()
{
    Stop();
}

void
KinectV1::Start()
{
	


}

void
KinectV1::Stop()
{
	std::clog << "KinectV1::Stop()" << std::endl;
}

//ColorPixels<ColorType>
//KinectV1::GetColorPixels()
//{
//	std::clog << "KinectV1::GetColorPixels()" << std::endl;
//
//    // ToDo: Ta kopia jest raczej nie potrzebna.
//    // To i tak ma byæ tylko do odczytu na bie¿¹co.
//    // I tak tylko na podgl¹d ma iœæ.
//    return ColorPixels<ColorType>(_kinect2grabber->GetColorWidth(), _kinect2grabber->GetColorHeight(), _kinect2grabber->GetColorBufferData());
//   
//}

Colors
KinectV1::GetColorPixels()
{
	std::clog << "KinectV1::GetColorPixelsPtr()" << std::endl;

    throw std::logic_error("Nie zaimplementowano");

    Colors colors =
    {
        //_kinect2grabber->GetColorWidth(),
        //_kinect2grabber->GetColorHeight(),
        //_kinect2grabber->GetColorBufferData()
    };

    return colors;
}

pcl::PointCloud<PointType>::ConstPtr
KinectV1::GetPointCloud()
{
    return _kinectV1Grabber->grabCloud();

}
