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

        VideoCapture _capture;


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



            



        }

        private void process_frame(VideoCapture capture_object)
        {

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
