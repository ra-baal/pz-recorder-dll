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

            var bitmap = manager.GetColorBitmap();
            Console.WriteLine(bitmap.Length);

            //foreach (var color in bitmap)
            //    Console.WriteLine($"b:{color.b} g:{color.g} r:{color.r}");

        }

        public static void RecordingManager_LivePreview_Test()
        {
            IRecordingManager manager = new RecordingManager();

            while (true)
            {
                var bitmap = manager.GetColorBitmap();
                int l = bitmap.Length;
                Console.WriteLine(l);
                (byte b, byte g, byte r) color = bitmap[l / 2];
                Console.WriteLine($"b:{color.b} g:{color.g} r:{color.r}");
                Thread.Sleep(200); 
            }


        }


    }



}

