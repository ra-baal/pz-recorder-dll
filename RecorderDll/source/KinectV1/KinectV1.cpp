#include "KinectV1.h"

#include <pcl/filters/voxel_grid.h>

#include "../additionals.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

KinectV1::KinectV1() 
{
	LOG("KinectV1::KinectV1()")

    try {
        _kinectV1Grabber = std::make_shared<KinectV1Grabber>();
    }
    catch (...)
    {
        throw DeviceNotFoundException("Kinect V1 not found");
    }
    
    if (_kinectV1Grabber == nullptr)
        throw DeviceNotFoundException("Kinect V1 not found");

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

Colors
KinectV1::GetColorPixels()
{
	//LOG("KinectV1::GetColorPixels()")

    try 
    {
        Colors colors
        (
            _kinectV1Grabber->GetColorWidth(),
            _kinectV1Grabber->GetColorHeight(),
            ColorFormat::RGB_888,
            _kinectV1Grabber->GetColorBufferData()
        );

        return colors;

    }
    catch (...)
    {
        // ToDo: To raczej niczym nie rzuca.

	    LOG("KinectV1::GetColorPixels() - catch")
    }

    return Colors();
}

// ToDo: do zmiany.
// Get nie mo�e polega� na tym, �e dopiero t� chmur� nagramy.
pcl::PointCloud<PointType>::ConstPtr
KinectV1::GetPointCloud()
{
    return _kinectV1Grabber->grabCloud();
}

pcl::PointCloud<PointType>::Ptr
KinectV1::PrepareCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    *cloud_ptr = *cloud;


    // flip
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate(Eigen::AngleAxisf(22 / 7.0, Eigen::Vector3f::UnitZ()));
    transformPointCloud(*cloud_ptr, *cloud_ptr, transform_2);


    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.02f, 0.02f, 0.02f);
    vg.filter(*cloud_ptr);

    return cloud_ptr;
}

void 
KinectV1::RecordOneFrame(std::string filepath)
{
    //LOG("KinectV1::RecordOneFrame(std::string filepath)")
    auto cloud = _kinectV1Grabber->grabCloud();
    auto cloud_ptr = PrepareCloud(cloud);
    pcl::io::savePCDFileASCII(filepath, *cloud_ptr);
}
