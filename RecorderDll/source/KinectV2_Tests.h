#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include "ICloudRecorder.h"
#include "KinectV2/KinectV2.h"

#include <pcl/io/pcd_io.h> // For pcl::io::savepcdfileascii

class Timer
{
    private:
        time_t _start;
        time_t _countdown;

    public:
        Timer(time_t countdown)
        {
            _countdown = countdown;
            _start = time(nullptr);
        }

        bool IsUp()
        {
            return time(nullptr) > _start + _countdown;
        }


};

void kinectV2_print_color_data()
{
    std::shared_ptr<ICloudRecorder> cloudRecorder = std::make_shared<KinectV2>();
	cloudRecorder->Start();

    std::cout << "Nakieruj na kolor pierwszy" << std::endl;
    Timer timer(5); // Record for given secs.
    while( !timer.IsUp() )
    {

    }


    
    auto colorPixels = cloudRecorder->GetColorPixels();


    if (colorPixels._data != nullptr)
    {
        byte* asbytes = (byte*)(colorPixels._data);
            
        for (int i = 0; i < 5; i++)
        {
            std::cout << "r:" << (unsigned int)asbytes[0] << " g:" << (unsigned int)asbytes[1] << " b:" << (unsigned int)asbytes[2] << " Reserved:" << (unsigned int)asbytes[3] << std::endl;
            asbytes += 4;
        }

    }


    std::cout << "Nakieruj na kolor drugi" << std::endl;

    Timer timer2(5); // Record for given secs.
    while( !timer2.IsUp() )
    {

    }


    auto colorPixels2 = cloudRecorder->GetColorPixels();


    if (colorPixels2._data != nullptr)
    {
        byte* asbytes = (byte*)(colorPixels2._data);
            
        for (int i = 0; i < 5; i++)
        {
            std::cout << "r:" << (unsigned int)asbytes[0] << " g:" << (unsigned int)asbytes[1] << " b:" << (unsigned int)asbytes[2] << " Reserved:" << (unsigned int)asbytes[3] << std::endl;
            asbytes += 4;
        }

    }


}

void kinectV2_save_files()
{
	std::shared_ptr<ICloudRecorder> cloudRecorder = std::make_shared<KinectV2>();
	cloudRecorder->Start();

    std::vector<std::string> filenames;

    int frame = 0;
    Timer timer(60); // Record for given secs.
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


