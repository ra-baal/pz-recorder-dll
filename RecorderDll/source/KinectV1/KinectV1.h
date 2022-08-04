#pragma once

#include "../ICloudRecorder.h" // For ICloudRecorder
#include "KinectV1Grabber.h"

class KinectV1 : public ICloudRecorder
{
public:
	KinectV1();
	~KinectV1();

	// Inherited via ICloudRecorder
	virtual void Start() override;
	virtual void Stop() override;
	virtual Colors GetColorPixels() override;
	//Colors GetColorPixelsPtr();
	virtual pcl::PointCloud<PointType>::ConstPtr GetPointCloud() override;
	virtual void RecordOneFrame(std::string filepath) override;

private: 
	std::shared_ptr<KinectV1Grabber> _kinectV1Grabber;

	// Mo¿e gromadziæ te chmury w kolejce?
	//Kolejka<Chmur> _chmury;
	pcl::PointCloud<PointType>::ConstPtr _pointCloud;
};
