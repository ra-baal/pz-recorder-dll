using System.Runtime.InteropServices;

namespace Recorder.Model
{
    public unsafe class RecordingManager : IRecordingManager
    {
        #region DllImport.
        private const string _dll = "RecorderDll.dll";
        [DllImport(_dll)] public static extern int RecordingManager_Test(int a, int b);
        [DllImport(_dll)] private static extern void* RecordingManager_New();
        [DllImport(_dll)] private static extern void RecordingManager_Delete(void* recordingManager);
        [DllImport(_dll)] private static extern int RecordingManager_GetRecordersNumber(void* recordingManager);
        [DllImport(_dll)] private static extern Colors RecordingManager_GetColorBitmap(void* recordingManager);
        [DllImport(_dll)] private static extern void RecordingManager_RecordingMode(void* recordingManager);
        [DllImport(_dll)] private static extern void RecordingManager_PreviewMode(void* recordingManager);
        #endregion

        #region Handlers.

        private void* _objptr;

        public RecordingManager()
        {
            _objptr = RecordingManager_New();

            if (_objptr == null)
                throw new ExternalException();
        } 

        public int GetRecordersNumber() => RecordingManager_GetRecordersNumber(_objptr);

        public (byte b, byte g, byte r)[] GetColorBitmap()
        {
            Colors colors = RecordingManager_GetColorBitmap(_objptr);

            (byte b, byte g, byte r)[] bitmap = new (byte r, byte g, byte b)[colors.Heigth * colors.Width];

            RGBQUAD* rgbquadPtr = colors.Data;
            for (int i = 0; i < colors.Width * colors.Heigth; i++)
            {
                byte* bytePtr = (byte*)rgbquadPtr; // RGBQUAD.rgbBlue
                bitmap[i].b = *bytePtr; 

                bytePtr++; // RGBQUAD.rgbGreen
                bitmap[i].g = *bytePtr;

                bytePtr++; // RGBQUAD.rgbRed
                bitmap[i].r = *bytePtr;

                bytePtr++; // RGBQUAD.rgbReserved

                rgbquadPtr++; // Następne 4 bajty
            }

            return bitmap;
        }

        public void RecordingMode() => RecordingManager_PreviewMode(_objptr);

        public void PreviewMode() => RecordingManager_PreviewMode(_objptr);

        #endregion


    }
}
