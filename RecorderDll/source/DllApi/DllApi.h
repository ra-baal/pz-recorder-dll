#pragma once
    
#ifdef RecorderDll_EXPORTS
#define RecorderDll_API __declspec(dllexport)
#else
#define RecorderDll_API __declspec(dllimport)
#endif

#include "../additionals.h"

extern "C" RecorderDll_API int Test();
    
extern "C" RecorderDll_API void* KinectV2_New();
extern "C" RecorderDll_API void KinectV2_Delete(void* kinectV2);
extern "C" RecorderDll_API void KinectV2_Start(void* kinectV2);
extern "C" RecorderDll_API void KinectV2_Stop(void* kinectV2);
//extern "C" RecorderDll_API void* KinectV2_GetColorPixels(void* kinectV2);
extern "C" RecorderDll_API Colors KinectV2_GetColorPixelsPtr(void* kinectV2);
//pcl::PointCloud<PointType>::ConstPtr KinectV2_GetPointCloud(void* kinectV2);
