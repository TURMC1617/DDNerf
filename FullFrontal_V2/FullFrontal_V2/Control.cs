using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using ArduinoDriver;
using ArduinoUploader;
using ArduinoUploader.Hardware;
using ArduinoDriver.SerialProtocol;

/// <summary>
/// Control methods and arduino interface
/// </summary>


namespace FullFrontal_V2
{
    public partial class MainWindow : Window
    {

        public const ArduinoModel Arduino = ArduinoModel.UnoR3;




        private void stop_shooting(ArduinoDriver.ArduinoDriver driver)
        {

            //set relay to low
        }

        private void start_shooting(ArduinoDriver.ArduinoDriver driver)
        {
            //set pin to high
        }


    }
}
