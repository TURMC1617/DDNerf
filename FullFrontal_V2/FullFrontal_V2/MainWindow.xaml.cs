using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Emgu.CV;
using ArduinoDriver;
using ArduinoUploader;

namespace FullFrontal_V2
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        //Camera Number:
        private static int cam_number;

        VideoCapture _capture;




        /// <summary>
        /// Start Button Click Handler
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Start_Click(object sender, RoutedEventArgs e)
        {

            if (camera_option.Text == "USB0")
            {
                cam_number = 0;
            }
            if (camera_option.Text == "USB1")
            {
                cam_number = 1;
            }

            Start.IsEnabled = false;

            _capture = new VideoCapture(cam_number);
                


        }
    }
}
