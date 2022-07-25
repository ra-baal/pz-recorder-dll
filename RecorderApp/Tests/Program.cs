using RecorderModel;
using System.Threading;

Console.WriteLine("Test KinectV2");


int x = KinectV2.TestDll();

Console.WriteLine($"x={x}");


KinectV2 kinectV2 = new KinectV2();

kinectV2.Start();

Thread.Sleep(5 * 1000);

Colors colors = kinectV2.GetColorPixelsPtr();

unsafe
{
    Console.WriteLine($"{colors.Width} {colors.Heigth}");
    Console.WriteLine($"r:{colors.Data->rgbRed} g:{colors.Data->rgbGreen} b:{colors.Data->rgbBlue} reserved: {colors.Data->rgbReserved}");
    Console.WriteLine($"r:{(colors.Data + 1)->rgbRed} g:{(colors.Data + 1)->rgbGreen} b:{(colors.Data + 1)->rgbBlue} reserved: {(colors.Data + 1)->rgbReserved}");
}

kinectV2.Stop();


