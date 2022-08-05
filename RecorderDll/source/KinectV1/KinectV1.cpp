#include "KinectV1.h"
#include "../additionals.h"
#include <pcl/io/pcd_io.h>

KinectV1::KinectV1() 
{
	LOG("KinectV1::KinectV1()")
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
	LOG("KinectV1::Stop()")
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
	LOG("KinectV1::GetColorPixels()")

    try 
    {
        Colors colors = {
            _kinectV1Grabber->GetColorWidth(),
            _kinectV1Grabber->GetColorHeight(),
            _kinectV1Grabber->GetColorBufferData()
        };
        return colors;
    }
    catch (...)
    {
	    LOG( "KinectV1::GetColorPixels() - catch")
    }

    return Colors();
}

// ToDo: do zmiany.
// Get nie mo¿e polegaæ na tym, ¿e dopiero tê chmurê nagramy.
pcl::PointCloud<PointType>::ConstPtr
KinectV1::GetPointCloud()
{
    return _kinectV1Grabber->grabCloud();
}

void 
KinectV1::RecordOneFrame(std::string filepath)
{
    LOG("KinectV1::RecordOneFrame(std::string filepath)")
    auto cloud = _kinectV1Grabber->grabCloud();
    pcl::io::savePCDFileASCII(filepath, *cloud);
}
