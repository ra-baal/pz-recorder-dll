using System.Threading;
using Recorder.Model;


namespace Recorder.ModelTests
{
    public class Program
    {

        public static int Main()
        {
            //RecordingManager_Test();
            RecordingManager_LivePreview_Test();

            return 0;
        }

        public static void RecordingManager_Test()
        {
            int x = RecordingManager.RecordingManager_Test(1, 2);
            Console.WriteLine(x);

            IRecordingManager manager = new RecordingManager();

            Thread.Sleep(5 * 1000); // Trzeba poczekać, żeby Kinect zaczął rejestrować. 

            (byte b, byte g, byte r)[][]? bitmaps = manager.GetColorBitmaps();
            
            Console.WriteLine($"bitmaps.Length:{bitmaps.Length}");
            Console.WriteLine($"bitmaps[0].Length:{bitmaps[0].Length}");
            Console.WriteLine($"bitmaps[1].Length:{bitmaps[1].Length}");

            //foreach (var color in bitmap)
            //    Console.WriteLine($"b:{color.b} g:{color.g} r:{color.r}");

            (byte b, byte g, byte r) color0 = bitmaps[0][0];
            (byte b, byte g, byte r) color1 = bitmaps[1][1];
            Console.WriteLine($"b:{color0.b} g:{color0.g} r:{color0.r}");
            Console.WriteLine($"b:{color1.b} g:{color1.g} r:{color1.r}");

        }

        public static void RecordingManager_LivePreview_Test()
        {
            IRecordingManager manager = new RecordingManager();
            Thread.Sleep(1000); // ToDo: trzeba doprowadzić do postaci, w której nie będzie potrzeba tych pauz.

            manager.StartRecording();
            Thread.Sleep(1000); // ToDo: trzeba doprowadzić do postaci, w której nie będzie potrzeba tych pauz.

            int i = 0;

            while (i < 10)
            {
                (byte b, byte g, byte r)[][]? bitmaps = manager.GetColorBitmaps();

                Console.WriteLine($"bitmaps.Length:{bitmaps.Length}");
                Console.WriteLine($"bitmaps[0].Length:{bitmaps[0].Length}");
                Console.WriteLine($"bitmaps[1].Length:{bitmaps[1].Length}");

                int len0 = bitmaps[0].Length;
                int len1 = bitmaps[1].Length;

                (byte b, byte g, byte r) color0 = bitmaps[0][len0 / 2];
                (byte b, byte g, byte r) color1 = bitmaps[1][len1 / 2];
                Console.WriteLine($"b:{color0.b} g:{color0.g} r:{color0.r}");
                Console.WriteLine($"b:{color1.b} g:{color1.g} r:{color1.r}");
                Thread.Sleep(200);
                i++;
            }

            manager.StopRecording();


        }


    }



}

