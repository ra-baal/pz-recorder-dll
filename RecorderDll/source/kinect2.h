#pragma once
    
#ifdef SAMPLE_EXPORTS
#define SAMPLE_API __declspec(dllexport)
#else
#define SAMPLE_API __declspec(dllimport)
#endif

extern "C" SAMPLE_API int kinect2_main( int argc, char* argv[] );

extern "C" SAMPLE_API double test(double a, double b);






