//#include "ICloudRecorder.h"
//#include "KinectV2/KinectV2.h"
#include "main_Tests.h"

int main()
{
	LOG("main: start")
	
	//kinect_v1_main();

    //kinectV2_visualization();
	//kinectV2_save_files(10);
	//kinectV2_print_color_data();
	//kinectV2_print_color_data_DllApi();

	//kinectV1_class_test();

	recordingManager_classTest();

	LOG("main: end")
	return 0;
}


