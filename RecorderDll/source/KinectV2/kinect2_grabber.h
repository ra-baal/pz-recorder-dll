/// Kod z https://github.com/UnaNancyOwen/KinectGrabber/tree/Kinect2Grabber
/// z niewielkimi modyfikacjami.
/// 
/// RB: Podsumowanie dokonanych zmian w tym pliku:
/// 1. boost::thread --> std::thread
///     - By³ problem z konstruktorem bezargumentowym?
/// 2. boost::shared_ptr --> std::shared_ptr
/// 
/// To chyba jedyne zmiany.

// Kinect2Grabber is pcl::Grabber to retrieve the point cloud data from Kinect v2 using Kinect for Windows SDK 2.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT2_GRABBER
#define KINECT2_GRABBER

#define NOMINMAX
 
#include <pcl/io/grabber.h> // For pcl::Grabber
#include <Windows.h> // For UINT16 i RGBQUAD
#include <Kinect.h> // For IKinectSensor and other interfaces
#include <pcl/point_types.h> // pcl::PointXYZ etc.
#include <pcl/point_cloud.h> // pcl::PointCloud etc.
#include <thread> // For std::thread
#include <boost/thread/mutex.hpp>
		

namespace pcl
{
    class Kinect2Grabber : public pcl::Grabber
    {
        public:
            Kinect2Grabber();
            virtual ~Kinect2Grabber() throw ();
            virtual void start();
            virtual void stop();
            virtual bool isRunning() const;
            virtual std::string getName() const;
            virtual float getFramesPerSecond() const;

            /// RB: Zamieni³em boost::shared_ptr na std::shared_ptr.
            typedef void ( signal_Kinect2_PointXYZ )( const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& );
            typedef void ( signal_Kinect2_PointXYZI )( const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& );
            typedef void ( signal_Kinect2_PointXYZRGB )( const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& );
            typedef void ( signal_Kinect2_PointXYZRGBA )( const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>& );

        protected:
            boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
            boost::signals2::signal<signal_Kinect2_PointXYZI>* signal_PointXYZI;
            boost::signals2::signal<signal_Kinect2_PointXYZRGB>* signal_PointXYZRGB;
            boost::signals2::signal<signal_Kinect2_PointXYZRGBA>* signal_PointXYZRGBA;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ( UINT16* depthBuffer );
            pcl::PointCloud<pcl::PointXYZI>::Ptr convertInfraredDepthToPointXYZI( UINT16* infraredBuffer, UINT16* depthBuffer );
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB( RGBQUAD* colorBuffer, UINT16* depthBuffer );
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA( RGBQUAD* colorBuffer, UINT16* depthBuffer );

            void threadFunction();

            // RB: Zamiana boost::thread na std::thread rozwi¹za³a problem.
            /*boost*/std::thread thread; 

            mutable boost::mutex mutex;

            bool quit;
            bool running;

            HRESULT result;
            IKinectSensor* sensor;
            ICoordinateMapper* mapper;
            IColorFrameSource* colorSource;
            IColorFrameReader* colorReader;
            IDepthFrameSource* depthSource;
            IDepthFrameReader* depthReader;
            IInfraredFrameSource* infraredSource;
            IInfraredFrameReader* infraredReader;

            int colorWidth;
            int colorHeight;
            std::vector<RGBQUAD> colorBuffer;

            int depthWidth;
            int depthHeight;
            std::vector<UINT16> depthBuffer;

            int infraredWidth;
            int infraredHeight;
            std::vector<UINT16> infraredBuffer;
    };

} // end namespace

#endif KINECT2_GRABBER

