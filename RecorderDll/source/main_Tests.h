#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include "ICloudRecorder.h"
#include "KinectV2/KinectV2.h"
#include "KinectV1/KinectV1.h"

#include <pcl/io/pcd_io.h> // For pcl::io::savepcdfileascii
#include <thread>

#include "RecordingManager/RecordingManager.h"

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
    LOG("0")

    RecordingManager manager;

    LOG("1")

    LOG("int GetRecordersNumber: " << manager.GetRecordersNumber())
    
    LOG("2")

    manager.StartRecording();

    LOG("3")

    Timer timer(10);

    LOG("4")

    timer.Wait();

    LOG("5")

    LOG( "Colors* GetColorBitmaps: " << manager.GetColorBitmaps() )
    auto data0 = manager.GetColorBitmaps()[0].Data;
    LOG( "(*manager.GetColorBitmaps()[0].Data).rgbRed: " << (data0 ? ((Rgb24*)data0)->r : -1) )
    LOG( "ColorFormat: " << manager.GetColorBitmaps()[0].Format )
    auto data1 = manager.GetColorBitmaps()[1].Data;
    LOG( "(*manager.GetColorBitmaps()[1].Data).rgbRed: " << (data1 ? ((Bgr32*)data1)->r : -1) )
    LOG( "ColorFormat: " << manager.GetColorBitmaps()[1].Format )

    LOG("6")

    manager.StopRecording();
    
    LOG("7")

}

void kinectV1_classTest()
{
    auto kinectV1 = std::make_shared<KinectV1>();

    auto cloud = kinectV1->GetPointCloud();

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


