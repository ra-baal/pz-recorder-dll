//#include "KinectV2/kinect_v2.h"
//#include "KinectV1/kinect_v1.h"
//#include "ICloudRecorder.h"
//#include "KinectV2/KinectV2.h"
#include "main_Tests.h"

// todos:
// - robiæ kopiê chmury na bie¿¹co i wrzucaæ do kolejki
// - nastêpnie oddzielny w¹tek (proces?) bêdzie to pokolei zapisywa³ do plików
// powinno to trochê przyspiezyc wtedy (frames per sec)

int main()
{
	std::clog << "main: start" << std::endl;
	
	//kinect_v1_main();

	//kinect_v2_main_UnaNancyOwen();
	//kinect_v2_main_wz18207();

    //kinectV2_visualization();
	//kinectV2_save_files(10);
	//kinectV2_print_color_data();
	//kinectV2_print_color_data_DllApi();

	//kinect_threads();

	//kinectV1_class_test();

	recordingManager_classTest();

	std::clog << "main: end" << std::endl;
	return 0;
}


