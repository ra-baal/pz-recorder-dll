

namespace Recorder.Model
{
    public interface IRecordingManager
    {
        public int GetRecordersNumber();
        //public RecorderState[] GetStates();
        //public WriteableBitmap GetColorBitmap();
        public (byte b, byte g, byte r)[] GetColorBitmap();
        public void RecordingMode();
        public void PreviewMode();

    }
}
