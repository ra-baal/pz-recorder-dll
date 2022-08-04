#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include "ICloudRecorder.h"
#include "KinectV2/KinectV2.h"
#include "KinectV1/kinect_v1.h"
#include "KinectV2/kinect_v2.h"
#include "KinectV1/KinectV1.h"

#include <pcl/io/pcd_io.h> // For pcl::io::savepcdfileascii
#include <thread>

#include "RecordingManager.h"

class Timer
{
    private:
        time_t _start;
        time_t _countdown;

    public:
        Timer(time_t seconds)
        {
            _countdown = seconds;
            _start = time(nullptr);
        }

        bool IsUp()
        {
            return time(nullptr) > _start + _countdown;
        }

        void Wait()
        {
            while (!IsUp());
        }


};

void recordingManager_classTest()
{
    std::cout << 0 << std::endl;

    RecordingManager manager;

    std::cout << 1 << std::endl;

    //std::cout << "Colors* GetColorBitmaps: " << manager.GetColorBitmaps() << std::endl;

    std::cout << 2 << std::endl;

    std::cout << "int GetRecordersNumber: " << manager.GetRecordersNumber() << std::endl;

    std::cout << 3 << std::endl;

    manager.RecordingMode();

    std::cout << 4 << std::endl;

    Timer timer(10);

    std::cout << 5 << std::endl;

    timer.Wait();

    std::cout << 6 << std::endl;

    manager.PreviewMode();

    std::cout << 7 << std::endl;

}

void kinectV1_classTest()
{
    auto kinectV1 = std::make_shared<KinectV1>();

    auto cloud = kinectV1->GetPointCloud();

}

void kinect_threads()
{
    std::thread thread_V1(kinect_v1_main);
    std::thread thread_V2(kinect_v2_main_UnaNancyOwen);
  
    thread_V1.join();
    thread_V2.join();
}

void kinectV2_save_files(int duration_seconds)
{
	std::shared_ptr<ICloudRecorder> cloudRecorder = std::make_shared<KinectV2>();
	cloudRecorder->Start();

    std::vector<std::string> filenames;

    int frame = 0;
    Timer timer(duration_seconds); // Record for given secs.
    while( !timer.IsUp() )
    {
        pcl::PointCloud<PointType>::ConstPtr cloud = cloudRecorder->GetPointCloud();

        if( cloud )
        {
            std::string filename = "ascii-frame" + std::to_string(frame) + ".pcd";
            pcl::io::savePCDFileASCII(filename, *cloud);
            filenames.push_back(filename);
            frame++;
        }

    }

    // settings.vrfilm
    std::ofstream outfile;
    outfile.open("settings.vrfilm", ios::out | ios::trunc );

    outfile << "pcd\n";
    outfile << std::to_string(frame) << '\n';

    for (std::string filename : filenames) 
        outfile << filename << '\n';

}

void kinectV2_visualization()
{
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    
	std::shared_ptr<ICloudRecorder> cloudRecorder = std::make_shared<KinectV2>();
	cloudRecorder->Start();
    std::cout << "Press space to finish visualization." << std::endl;

    while( !viewer->wasStopped() )
    {
        // Update Viewer
        viewer->spinOnce();

        pcl::PointCloud<PointType>::ConstPtr cloud = cloudRecorder->GetPointCloud();

        // Dzia³a to bez tego locka?
        // Byæ mo¿e przez to 
        //boost::mutex::scoped_try_lock lock( mutex );
        if( /*lock.owns_lock() &&*/ cloud )
        {
            // Update Point Cloud
            if( !viewer->updatePointCloud( cloud, "cloud" ) )
            {
                viewer->addPointCloud( cloud, "cloud" );

            }
        }

        if(GetKeyState(VK_SPACE) & 0x8000)
            break;

    }

}


