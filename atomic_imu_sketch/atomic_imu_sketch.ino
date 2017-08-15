
byte incoming_data[16];
unsigned int imu_data[6];


void setup() {
  // put your setup code here, to run once:


  Serial.begin(115200);
  Serial.flush();
  Serial.write(41); //Write ASCII 41 => to byte: ")" sets sample frequency to 50HZ; testing purposes only

  delay(10);

  Serial.write(35); //command IMU to start sending data
  Serial.print("Begin");

}

void loop() {
  // put your main code here, to run repeatedly:


  for (int i=0; i<16; i++)
  {
    if (Serial.available())
    {
      incoming_data[i] = Serial.read();
      //Serial.print(incoming_data[i]);//read raw incoming data from IMU byte to byte
      
    }
  }

  //Now time to convert BYTES into Ints/readible usable information

  for (int j=0; j<6; j++)
  {
    //high byte of imu data is MSB = 1'st of 2 bytes recieved from IMU, index into rawData to skip "A" and count

    unsigned int x_high = (unsigned int)incoming_data[2*j+3];
    unsigned int x_low = (unsigned int)incoming_data[2*j+4];
    
    
    int combined = multiply_combine(x_high, x_low);

    imu_data[j] = combined;
    
  }

  //Serial.begin(115200);
  //Serial.print(incoming_data[0]);
  //Serial.print(",");
  for (int k = 0; k<6; k++)
  {
    Serial.print(imu_data[k]);
    Serial.print(",");
    
  }
  Serial.print("\n");

  //delay(100);


}

int multiply_combine(unsigned int x_high, unsigned int x_low)
{
  int combined;
  combined = x_high;
  combined = combined * 256;
  combined |= x_low;
  return combined;
}

