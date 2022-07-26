

namespace Recorder.Model
{
    // Typ identyczny jak w Windows.h -> wingdi.h
    public struct RGBQUAD
    {
        public byte rgbBlue;
        public byte rgbGreen;
        public byte rgbRed;
        public byte rgbReserved;
    }

    // Typ identyczny jak w RecorderDll -> additionals.h
    public unsafe struct Colors
    {
        public int Width;
        public int Heigth;
        public RGBQUAD* Data;
    }
}
