#include <Wire.h>
#include<I2Cdev.h>
#include<MPU6050.h>
 
MPU6050 mpu;
float pitch=0;
float roll=0;
float yaw=0;
float v_pitch;
float v_roll;
float v_yaw;
float a_pitch;
float a_roll;
float a_yaw;
int16_t ax, ay, az;
int16_t gx, gy, gz;
 
#define pin1 3
#define pin2 5
 
void setup(){
 Serial.begin(9600);
 Serial.println("Initialize MPU");
 mpu.initialize();
 Serial.println(mpu.testConnection() ? "Connected" : "Connection failed"); 
 pinMode(pin1,OUTPUT);
 pinMode(pin2,OUTPUT);
 mpu.setXAccelOffset(-1812);
 mpu.setYAccelOffset(1379);
 mpu.setZAccelOffset(1427);
 mpu.setXGyroOffset(77);
 mpu.setYGyroOffset(-47);
 mpu.setZGyroOffset(64);
 
}
 
void loop(){
 mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
v_pitch=(gx/131);
  if(v_pitch==-1)
//error filtering
{
  v_pitch=0;
  }
  v_roll=(gy/131);
  if(v_roll==1)
  //error filtering
  {v_roll=0;}
  v_yaw=gz/131;
  a_pitch=(v_pitch*0.046);
  a_roll=(v_roll*0.046);
  a_yaw=(v_yaw*0.045);
  pitch= pitch + a_pitch;
  roll= roll + a_roll;
  yaw= yaw + a_yaw; 


  Serial.print(" | pitch = ");
  Serial.print(pitch);
  Serial.print(" | roll = ");
  Serial.print(roll);
  Serial.print(" | yaw = ");
  Serial.println(yaw);

 delay(500);
}
