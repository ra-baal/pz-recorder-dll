/// Kod z https://github.com/UnaNancyOwen/KinectGrabber/tree/Kinect2Grabber
/// z niewielkimi modyfikacjami.
///
/// RB: By³o generowanych setki b³êdów w bibliotekach.
/// Mo¿liwe, ¿e ca³y problem stanowi³ boost::function.
/// Dokona³em zamiany boost::function na std::function,
/// poniewa¿ metoda pcl::Grabber::registerCallback przyjmowa³a tylko std::function.
/// Pojawia siê pytanie, dlaczego tak by³o.
/// Byæ mo¿e zmieni³a siê sygnatura tej funkcji w nowszej wersji PCL?

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <pcl/point_types.h> // For pcl::PointXYZRGBA
#include <boost/shared_ptr.hpp> // For boost::shared_ptr
#include <pcl/visualization/pcl_visualizer.h> // For pcl::visualization::PCLVisualizer

#include <boost/thread/mutex.hpp> // For boost::mutex

#include <boost/make_shared.hpp> // For boost::make_shared

#include <pcl/io/grabber.h> // For pcl::Grabber
#include "kinect2_grabber.h" // For pcl::Kinect2Grabber

#include <WinUser.h> // Sprawdzanie czy klawisz wciœniêty

#include "kinect2.h"

typedef pcl::PointXYZRGBA PointType;


double test(double a, double b)
{
    return a * b;
}


int kinect2_main( int argc, char* argv[] )
{
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;
    
    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;

    // RB: boost::function zosta³o zamienione na std::function.
    /*boost*/std::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr& ptr )
        {
            boost::mutex::scoped_lock lock( mutex );

            /* Point Cloud Processing */

            cloud = ptr->makeShared();
        };

    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
    
    boost::signals2::connection connection;
    try
    {
        // Register Callback Function
        connection = grabber->registerCallback( function );
        std::cout << "grabber->registerCallback successful :)" << std::endl;
    }
    catch (pcl::IOException& e)
    {
        std::cout << "grabber->registerCallback unsuccessful :(" << endl;
        std::cout << "what: " << e.what() << endl;
        //std::cout << endl;
        //std::cout << "detailedMessage: " << e.detailedMessage() << endl;
        //std::cout << endl;
        //std::cout << "getFileName: " << e.getFileName() << endl;
        //std::cout << endl;
        //std::cout << "getFunctionName: " << e.getFunctionName() << endl;
        //std::cout << endl;
        //std::cout << "getLineNumber: " << e.getLineNumber() << endl;
    
    }

    // Start Grabber
    grabber->start();

    std::cout << "Press space to finish." << std::endl;
    while( !viewer->wasStopped() )
    {
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud )
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

    // Stop Grabber
    grabber->stop();
    
    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }


    std::cout << "end" << std::endl;
    return 0;
}

