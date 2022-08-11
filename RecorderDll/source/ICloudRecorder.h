#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "additionals.h"

class ICloudRecorder
{
public:
	virtual void Start() = 0;
	virtual void Stop() = 0;
	virtual Colors GetColorPixels() = 0;
	virtual pcl::PointCloud<PointType>::ConstPtr GetPointCloud() = 0;
	virtual pcl::PointCloud<PointType>::Ptr PrepareCloud(const pcl::PointCloud<PointType>::ConstPtr&) = 0;

    virtual void RecordOneFrame(std::string filepath) = 0;


};

