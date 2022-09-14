

namespace Recorder.Model
{
    #region Typy identyczne z tymi w RecorderDll -> additionals.h.

    public struct Rgb24 
    {
        public byte r = 255;
        public byte g = 255;
        public byte b = 255;

        public Rgb24()
        {
        }
    }

    public struct Bgr32 
    {
        public byte b = 255;
        public byte g = 255;
        public byte r = 255;
        public byte _reserved = 0;

        public Bgr32()
        {
        }
    }

    public enum ColorFormat
    {
        UnknownFormat = 0,
        Rgb24 = 200, // (red, green, blue) ; 3*8 bitów = 24 bity = 3 bajty
        Bgr32 = 400 // RGBQUAD ; (blue, green, red, nieużywane) ; 4*8 bitów = 32 bity = 4 bajty (z czego tylko 3 wykorzystane)
    };

    // Typ identyczny jak w RecorderDll -> additionals.h
    public unsafe struct Colors
    {
        public int Width;
        public int Height;
        public ColorFormat Format;
        public void* Data;
    }

    #endregion
}
