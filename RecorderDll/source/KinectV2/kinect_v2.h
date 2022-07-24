#pragma once
    
#ifdef SAMPLE_EXPORTS
#define SAMPLE_API __declspec(dllexport)
#else
#define SAMPLE_API __declspec(dllimport)
#endif

extern "C" SAMPLE_API void kinect_v2_main_UnaNancyOwen();
extern "C" SAMPLE_API void kinect_v2_main_wz18207();

#include "kinect2_grabber.h"

//// test
//class KinectV2UnaNancyOwen
//{
//    private:
//        std::shared_ptr<pcl::Grabber> grabber;
//
//    public:
//        KinectV2UnaNancyOwen();
//
//        void Start();
//
//};
//
//
