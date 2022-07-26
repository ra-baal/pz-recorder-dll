//using System.Runtime.InteropServices;
////using System.Windows.Media.Imaging;

//namespace Recorder.Model
//{ 

//    public unsafe class KinectV2
//    {
//        #region DllImport.

//        private const string _dll = "RecorderDll.dll";

//        [DllImport(_dll)]
//        private static extern int Test();

//        [DllImport(_dll)]
//        private static extern void* KinectV2_New();

//        [DllImport(_dll)]
//        private static extern void KinectV2_Delete(void* kinectV2);

//        [DllImport(_dll)]
//        private static extern void KinectV2_Start(void* kinectV2);

//        [DllImport(_dll)]
//        private static extern void KinectV2_Stop(void* kinectV2);

//        [DllImport(_dll)]
//        private static extern Colors KinectV2_GetColorPixelsPtr(void* kinectV2);

//        #endregion

//        #region Handler methods.

//        private unsafe void* _objptr; // Object pointer.

//        public static int TestDll()
//        {
//            return Test();
//        }

//        public KinectV2()
//        {
//            _objptr = KinectV2_New();
//        }

//        public void Start()
//        {
//            KinectV2_Start(_objptr);
//        }

//        public void Stop()
//        {
//            KinectV2_Stop(_objptr);
//        }

//        public RGBQUAD[] GetColorPixelsPtr()
//        {
//            Colors colors = KinectV2_GetColorPixelsPtr(_objptr);
//            RGBQUAD[] pixels = new RGBQUAD[colors.Width * colors.Heigth];


//            //WriteableBitmap ColorBitmap;
//            //ColorBitmap.WritePixels(new System.Windows.Int32Rect(),)


//            return null;
//        }

//        ~KinectV2()
//        {
//            KinectV2_Delete(_objptr);
//        }

//        #endregion

//    }

//}
