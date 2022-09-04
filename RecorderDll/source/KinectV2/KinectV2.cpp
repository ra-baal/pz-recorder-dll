#include "KinectV2.h"

#include <pcl/filters/voxel_grid.h>

#include "KinectV2Grabber.h"
#include <pcl/io/pcd_io.h>
#include "../additionals.h"


KinectV2::KinectV2() 
{
	LOG("KinectV2::KinectV2()")

    try {
	    _kinect2grabber = std::make_shared<pcl::Kinect2Grabber>();

    }
    catch (std::exception& e)
    {
        throw;
        //throw DeviceNotFoundException(e.what());
    }
    catch (...)
    {
        throw DeviceNotFoundException("Kinect V2 not found - catch(...)");
    }
    
    if (_kinect2grabber == nullptr)
    {
        throw DeviceNotFoundException("Kinect V2 not found - nullptr");
    }

	// RB: Co� jest nie tak w implementacji Kinect2Grabber 
	// i w przypadku gdy utworzy si� jego obiekt,
	// a nast�pnie bez wystartowania obiekt zostanie zwolniony,
	// to wywala program. Dlatego te� na razie od razu robi� start.
	//_kinect2grabber->start(); // przeniesino do Start(), ale nadal musi by� w konstruktorze.
    Start();


}

KinectV2::~KinectV2()
{
    Stop();

    if (_connection.connected())
        _connection.disconnect();
}

void
KinectV2::Start()
{
    LOG("KinectV2::Start()")

    _kinect2grabber->start();

    // Retrieved Point Cloud Callback Function
	std::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> callbackFunction =
        [this]( const pcl::PointCloud<PointType>::ConstPtr& ptr )
        {
            //LOG("KinectV2::Start() - callbackFunction")
            
            // Sekcja krytyczna. //

            boost::mutex::scoped_lock lock( this->_mutex ); 
            
            /* Point Cloud Processing */

            this->_pointCloud = ptr->makeShared();

            // Koniec sekcji krytycznej. //
        };

    try
    {
        LOG("KinectV2::Start() - try")

        _connection = _kinect2grabber->registerCallback( callbackFunction );
        LOG("_kinect2grabber->registerCallback successful :)")
    }
    catch (pcl::IOException& e)
    {
        LOG("_kinect2grabber->registerCallback unsuccessful :(")
        LOG("what: " << e.what())
        throw DeviceNotFoundException("registerCallback failed in Kinect V2"); // Ma to sens tylko dop�ki metoda start jest wywo�ywana w konstruktorze.
    }
    catch(...)
    {
        LOG("KinectV2::Start() - catch(...)")
    }

}

void
KinectV2::Stop()
{
	LOG("KinectV2::Stop()")

    _kinect2grabber->stop();

}

Colors
KinectV2::GetColorPixels()
{
	//LOG("KinectV2::GetColorPixelsPtr()")

    Colors colors
    (
        _kinect2grabber->GetColorWidth(),
        _kinect2grabber->GetColorHeight(),
        ColorFormat::BGR32,
        _kinect2grabber->GetColorBufferData()
    );

    return colors;
}

pcl::PointCloud<PointType>::ConstPtr
KinectV2::GetPointCloud()
{
	//std::clog << "KinectV2::GetPointCloud()" << std::endl;

    // Sekcja krytyczna. //
    // Je�li dobrze rozumiem, to tutaj tylko pr�bujemy za�o�y� blokad�,
    // I uda to si� tylko wtedy gdy nie ma jej za�o�onej w funkcji callback.
    // Poniewa� nie chcemy ogranicza� mo�liwo�ci przechwytywania kolejnych chmur z kinecta.
    boost::mutex::scoped_try_lock lock(_mutex);
    if (lock.owns_lock())
    {
	    LOG("owns")
        return _pointCloud;
    }
    // Koniec sekcji krytycznej. //
    else
    {
	    LOG("not owns")
        return nullptr;
    }

    // Za ma�o klatek na sek si� zapisuje. Lepiej kopiowa� w pami�ci chmury i wrzuca� do kolejki
    // a w oddzielnym w�tku/procesie zapisywa� dopiero po kolei na dysk.

}

pcl::PointCloud<PointType>::Ptr
KinectV2::PrepareCloud(const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    *cloud_ptr = *cloud;

    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.02f, 0.02f, 0.02f);
    vg.filter(*cloud_ptr);

    return cloud_ptr;
}

void 
KinectV2::RecordOneFrame(std::string filepath)
{
    // Czy nie nale�a�oby zrobi� tutaj jakiej� blokady, mutexa ?
    auto cloud = _pointCloud;
    if( cloud != nullptr )
    {
        //LOG("KinectV2::RecordOneFrame(std::string filepath) - cloud is ok")
        auto cloud_ptr = PrepareCloud(cloud);
        pcl::io::savePCDFileASCII(filepath, *cloud_ptr);
    }
    else
    {
        LOG("KinectV2::RecordOneFrame(std::string filepath) - cloud is nullptr")
    }

}
