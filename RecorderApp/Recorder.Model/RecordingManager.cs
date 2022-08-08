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
        [DllImport(_dll)] private static extern Colors* RecordingManager_GetColorBitmaps(void* recordingManager);
        [DllImport(_dll)] private static extern void RecordingManager_StartRecording(void* recordingManager);
        [DllImport(_dll)] private static extern void RecordingManager_StopRecording(void* recordingManager);
        #endregion

        #region Handlers.

        private void* _objptr;

        public RecordingManager()
        {
            _objptr = RecordingManager_New();

            if (_objptr == null)
                throw new ExternalException("Nie udało się utworzyć obiektu po stronie biblioteki");
        } 

        public int GetRecordersNumber() => RecordingManager_GetRecordersNumber(_objptr);

        private (byte b, byte g, byte r)[] colorsToBgrArray(Colors colors)
        {
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

        public (byte b, byte g, byte r)[][] GetColorBitmaps()
        {
            Colors* colors = RecordingManager_GetColorBitmaps(_objptr);

            // 1
            (byte b, byte g, byte r)[] bitmap0 = colorsToBgrArray(colors[0]);

            // 2
            (byte b, byte g, byte r)[] bitmap1 = colorsToBgrArray(colors[1]);

            return new (byte b, byte g, byte r)[][]
            {
                bitmap0,
                bitmap1
            };
        }

        public void StartRecording() => RecordingManager_StartRecording(_objptr);

        public void StopRecording() => RecordingManager_StopRecording(_objptr);

        #endregion


    }
}
