

namespace Recorder.Model
{
    public interface IRecordingManager
    {
        //public RecorderState[] GetStates();
        public int GetRecordersNumber();
        public (byte b, byte g, byte r)[][] GetColorBitmaps();
        public void StartRecording();
        public void StopRecording();
        public void SetDirectory(string directory);

    }

}
