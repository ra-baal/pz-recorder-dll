#pragma once

#include "../ICloudRecorder.h" // For ICloudRecorder
#include "kinect2_grabber.h" // For pcl::Kinect2Grabber


class KinectV2 : public ICloudRecorder
{
public:
	KinectV2();
	~KinectV2();

	// Inherited via ICloudRecorder
	virtual void Start() override;
	virtual void Stop() override;
	virtual Colors GetColorPixels() override;
	//Colors GetColorPixelsPtr();
	virtual pcl::PointCloud<PointType>::ConstPtr GetPointCloud() override;
	virtual void RecordOneFrame(std::string filepath) override;

private: 
	std::shared_ptr<pcl::Kinect2Grabber> _kinect2grabber;

	// Mo¿e gromadziæ te chmury w kolejce?
	//Kolejka<Chmur> _chmury;
	pcl::PointCloud<PointType>::ConstPtr _pointCloud;
	boost::signals2::connection _connection;
	boost::mutex _mutex;

};
