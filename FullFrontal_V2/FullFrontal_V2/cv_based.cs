using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Timers;
using System.IO;
using System.Windows.Threading;
using Emgu.CV;
using System.Drawing;
namespace FullFrontal_V2
{
    public partial class MainWindow : Window
    {

        //Camera Number:
        public static int cam_number;

        private VideoCapture _capture;
        private CascadeClassifier _cascade;
        DispatcherTimer _timer;


        /// <summary>
        /// Start Button Click Handler
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CV_Start_Click(object sender, RoutedEventArgs e)
        {

            if (camera_option.Text == "USB0")
            {
                cam_number = 0;
            }
            if (camera_option.Text == "USB1")
            {
                cam_number = 1;
            }

            CV_Start.IsEnabled = false;

            _capture = new VideoCapture(cam_number);
            //_cascade = new CascadeClassifier(@"haarcascade_frontalface_default.xml");
            _timer = new DispatcherTimer();

            _timer.Tick += new EventHandler(timer_Tick);
            _timer.Interval = new TimeSpan(0, 0, 0, 0, 1);
            _timer.Start();








        }

        private void process_frame(VideoCapture capture_object, CascadeClassifier cascade, DispatcherTimer timer)
        {


        }


        void timer_Tick(object sender, EventArgs e)
        {
            Mat frame = _capture.QueryFrame();

            if (frame != null)
            {
                Mat gray = new Mat();
                CvInvoke.CvtColor(frame, gray, Emgu.CV.CvEnum.ColorConversion.Bgr2Gray);

                CV_Output.Source = BitmapToImageSource(gray.Bitmap);
            }
        }




        //Use this to convert mat frames into bitmaps so we can display
        BitmapImage BitmapToImageSource(Bitmap bitmap)
        {
            using (MemoryStream memory = new MemoryStream())
            {
                bitmap.Save(memory, System.Drawing.Imaging.ImageFormat.Bmp);
                memory.Position = 0;
                BitmapImage bitmapimage = new BitmapImage();
                bitmapimage.BeginInit();
                bitmapimage.StreamSource = memory;
                bitmapimage.CacheOption = BitmapCacheOption.OnLoad;
                bitmapimage.EndInit();
                return bitmapimage;
            }

        }

    }
}
