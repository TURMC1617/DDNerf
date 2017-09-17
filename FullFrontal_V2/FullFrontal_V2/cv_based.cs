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
using ArduinoDriver;
using ArduinoUploader.Hardware;
using ArduinoUploader;
using Priority_Queue;
using System.Threading;

namespace FullFrontal_V2
{
    public partial class MainWindow : Window
    {

        //Camera Number:
        public static int cam_number;

        private VideoCapture _capture;
        private CascadeClassifier _cascade;
        DispatcherTimer _timer;



        //1x2 (x and y coords) ordered based on closet distance to center of barrel
        public SimplePriorityQueue<Vector> fire_queue;


        //frame dimensions for Dell XPS 13 webcam
        public Vector cam_dim = new Vector(640, 480);
        //frame dimensions for frank.png test image
        public Vector image_dim = new Vector(929, 1200);

        

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

            

            //_capture = new VideoCapture(cam_number);
            _cascade = new CascadeClassifier(@"C:\Users\Merc.MERCURY\Documents\DDNerf\FullFrontal_V2\FullFrontal_V2\haarcascade_frontalface_default.xml");
            _timer = new DispatcherTimer();

            _timer.Tick += new EventHandler(timer_Tick);
            _timer.Interval = new TimeSpan(0, 0, 0, 0, 1);
            _timer.Start();








        }



        void timer_Tick(object sender, EventArgs e)
        {
            //Mat frame = _capture.QueryFrame();

            Mat frame = CvInvoke.Imread(@"C:\Users\Merc.Mercury\Desktop\franks.png", Emgu.CV.CvEnum.ImreadModes.AnyColor);

            if (frame != null)
            {
                Mat gray = new Mat();
                CvInvoke.CvtColor(frame, gray, Emgu.CV.CvEnum.ColorConversion.Bgr2Gray);
                CvInvoke.EqualizeHist(gray, gray);
                Rectangle[] faces = _cascade.DetectMultiScale(gray, 1.3, 3, new System.Drawing.Size(gray.Width / 30, gray.Height / 30), new System.Drawing.Size((int)((double)gray.Width / 1.05), (int)((double)gray.Height / 1.05)));

                Thread oThread = new Thread(new ParameterizedThreadStart(priority_sort));
                oThread.Start(faces);

               
                foreach (Rectangle face in faces)
                {
                    Emgu.CV.Structure.MCvScalar color = new Emgu.CV.Structure.MCvScalar(0, 0, 153);
                    CvInvoke.Rectangle(frame, face, color, 5);
                    
                    
                }

                CV_Output.Source = BitmapToImageSource(frame.Bitmap);
            }
        }


        double distance_formula(double X1, double Y1, double X2, double Y2)
        {
            double distance = System.Math.Sqrt(Math.Pow((X2 - X1), 2) + Math.Pow((Y2 - Y1), 2));


            return distance;
        }


        //Parameterized ThreadStart Handle
        private void priority_sort(object faces)
        {
            Rectangle[] foobar = (Rectangle[])faces;
            priority_sort(foobar);
        }

        //Enqueue Function to sort through faces
        private void priority_sort(Rectangle[] faces)
        {

            List<double> raw_list = new List<double>();
            //image.dim (width, height) or (X,Y)
            Vector center = new Vector(image_dim.X / 2, image_dim.Y / 2);

           
            foreach (Rectangle face in faces)
            {

                double face_distance = distance_formula(image_dim.X, image_dim.Y, face.X, face.Y);

                raw_list.Add(face_distance);

            }


            Console.WriteLine(raw_list);

            //A*,enqueue, pass to Control.cs
            
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
