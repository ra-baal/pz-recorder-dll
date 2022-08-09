

namespace Recorder.Model
{
    // Typ identyczny jak w Windows.h -> wingdi.h
    // Tożsame z BGR32.
    public struct RGBQUAD
    {
        public byte rgbBlue;
        public byte rgbGreen;
        public byte rgbRed;
        public byte rgbReserved;
    }

    // Identycznie jak w RecorderDll -> additionals.h
    public enum PixelFormat
    {
        UnknownFormat = 0,
        RGB_888 = 200, // (red, green, blue) ; 3*8 bitów = 24 bity = 3 bajty
        BGR32 = 400 // RGBQUAD ; (blue, green, red, nieużywane) ; 4*8 bitów = 32 bity = 4 bajty (z czego tylko 3 wykorzystane)
    };

    // Typ identyczny jak w RecorderDll -> additionals.h
    public unsafe struct Colors
    {
        public int Width;
        public int Heigth;
        public PixelFormat Format;
        public void* Data;
    }
}
