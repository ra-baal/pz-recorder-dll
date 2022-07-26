#include "KinectV2.h"
#include "kinect_v2.h"

KinectV2::KinectV2() 
{
	std::clog << "KinectV2::KinectV2()" << std::endl;
	_kinect2grabber = std::make_shared<pcl::Kinect2Grabber>();

	// RB: Co� jest nie tak w implementacji Kinect2Grabber 
	// i w przypadku gdy utworzy si� jego obiekt,
	// a nast�pnie bez wystartowania obiekt zostanie zwolniony,
	// to wywala program. Dlatego te� na razie od razu robi� start.
	_kinect2grabber->start();

}

KinectV2::~KinectV2()
{
    Stop();
}

void
KinectV2::Start()
{
	std::clog << "KinectV2::Start()" << std::endl;

    // Retrieved Point Cloud Callback Function
	std::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> callbackFunction =
        [this]( const pcl::PointCloud<PointType>::ConstPtr& ptr )
        {
            // Sekcja krytyczna. //
            // Je�li dobrze rozumiem, to tutaj bezwzgl�dnie zawsze zak�adamy blokad�,
            // poniewa� nie chcemy ogranicza� mo�liwo�ci przechwytywania kolejnych chmur z kinecta.
            boost::mutex::scoped_lock lock( this->_mutex ); 
            
            /* Point Cloud Processing */
            // todo: Tutaj kopiowa� do kolejki?

            this->_pointCloud = ptr->makeShared();

            //pcl::io::savePCDFileASCII("ascii-frame" + std::to_string(frame), *cloud);
            //pcl::io::savePCDFileBinary("binary-frame" + std::to_string(frame), *cloud);
            //frame++;

            // Koniec sekcji krytycznej. //
        };

    try
    {
        _connection = _kinect2grabber->registerCallback( callbackFunction );
        std::clog << "_kinect2grabber->registerCallback successful :)" << std::endl;
    }
    catch (pcl::IOException& e)
    {
        std::clog << "_kinect2grabber->registerCallback unsuccessful :(" << std::endl;
        std::clog << "what: " << e.what() << std::endl;
    
    }

	// Na razie metoda start jest uruchamiana w konstruktorze.
	// _kinect2grabber->start();


}

void
KinectV2::Stop()
{
	std::clog << "KinectV2::Stop()" << std::endl;

    _kinect2grabber->stop();
    
    if (_connection.connected())
        _connection.disconnect();

}

//ColorPixels<ColorType>
//KinectV2::GetColorPixels()
//{
//	std::clog << "KinectV2::GetColorPixels()" << std::endl;
//
//    // ToDo: Ta kopia jest raczej nie potrzebna.
//    // To i tak ma by� tylko do odczytu na bie��co.
//    // I tak tylko na podgl�d ma i��.
//    return ColorPixels<ColorType>(_kinect2grabber->GetColorWidth(), _kinect2grabber->GetColorHeight(), _kinect2grabber->GetColorBufferData());
//   
//}

Colors
KinectV2::GetColorPixels()
{
	std::clog << "KinectV2::GetColorPixelsPtr()" << std::endl;

    Colors colors =
    {
        _kinect2grabber->GetColorWidth(),
        _kinect2grabber->GetColorHeight(),
        _kinect2grabber->GetColorBufferData()
    };

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
    //boost::mutex::scoped_try_lock lock(_mutex);
    //if (lock.owns_lock())
    //{
	//    std::clog << "owns" << std::endl;
        return _pointCloud;
    //}
    // Koniec sekcji krytycznej. //
    //else
    //{
	//    std::clog << "not owns" << std::endl;
    //    return nullptr;
    //}

    // Za ma�o klatek na sek si� zapisuje. Lepiej kopiowa� w pami�ci chmury i wrzuca� do kolejki
    // a w oddzielnym w�tku/procesie zapisywa� dopiero po kolei na dysk.

}
