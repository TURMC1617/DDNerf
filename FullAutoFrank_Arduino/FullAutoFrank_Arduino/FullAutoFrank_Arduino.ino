#include <I2Cdev.h>
#include <MPU6050.h>
#include <helper_3dmath.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

/*
 Name:		FullAutoFrank_Arduino.ino
 Created:	2/17/2018 4:47:19 PM
 Author:	Brian Amin, Riley Mchugh, Tommy Tan, Eric Huang, Spencer Melnick
*/

#define MOTOR_PIN 1
#define RELAY_PIN 3
#define MAX_PWM 255
#define COSMOS
#define IMU_PIN1 3
#define IMU_PIN2 5

Servo motor;



float desired_angle;

//IMU global variables
bool IMU_updated = false;

bool power_off = true;

MPU6050 mpu;
float pitch = 0;
float roll = 0;
float yaw = 0;
float v_pitch;
float v_roll;
float v_yaw;
float a_pitch;
float a_roll;
float a_yaw;
int16_t ax, ay, az;
int16_t gx, gy, gz;



struct cosmos_t
{
	uint8_t length;
	uint8_t id;
	float desired_angle;
	float current_angle;
	int motor_power;
	float p;
	float i;
	float d;
}cosmos;

//PID Jazz
float kp = 2.5;
float ki = 4.0;
float kd = 25.0;

float error_test = 0.0;

double Setpoint;
double Input;
double Output;

PID pid_controller(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_M, DIRECT);
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior
/*
Input: The variable we're trying to control (double)
Output: The variable that will be adjusted by the pid (double)
Setpoint: The value we want to Input to maintain (double)
Kp, Ki, Kd: Tuning Parameters. these affect how the pid will chage the output. (double>=0)
Direction: Either DIRECT or REVERSE. determines which direction the output will move when faced with a given error. DIRECT is most common.
POn: Either P_ON_E (Default) or P_ON_M. Allows Proportional on Measurement to be specified.
*/

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(576000);
	motor.attach(MOTOR_PIN);
	mpu.initialize();
	pinMode(IMU_PIN1, OUTPUT);
	pinMode(IMU_PIN2, OUTPUT);
	//pinMode(MOTOR_PIN, OUTPUT);
	pinMode(RELAY_PIN, OUTPUT);
	mpu.setXAccelOffset(-1812);
	mpu.setYAccelOffset(1379);
	mpu.setZAccelOffset(1427);
	mpu.setXGyroOffset(77);
	mpu.setYGyroOffset(-47);
	mpu.setZGyroOffset(64);

}

// the loop function runs over and over again until power down or reset
void loop() {

	


	//check_serial();
	//update_IMU();
	//run_motors();
  
}

void update_IMU()
{
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	v_pitch = (gx / 131);
	if (v_pitch == -1)
		//error filtering
	{
		v_pitch = 0;
	}
	v_roll = (gy / 131);
	if (v_roll == 1)
		//error filtering
	{
		v_roll = 0;
	}
	v_yaw = gz / 131;
	a_pitch = (v_pitch*0.046);
	a_roll = (v_roll*0.046);
	a_yaw = (v_yaw*0.045);
	pitch = pitch + a_pitch;
	roll = roll + a_roll;
	yaw = yaw + a_yaw;

	IMU_updated = true;

}

void run_motors()
{
	static uint32_t last_update_time = millis();

	uint32_t update_time = millis();
	//turn off all motors
	if (power_off)
	{
		motor.write(90);

	}
	else if (IMU_updated)
	{
		float error = IMU_adjusted.yaw - desired_angle;
		float motor_power = getPID(error);
		error_test = error;

		cosmos.motor_power = motor_power;
		cosmos.current_angle = IMU_adjusted.yaw;
		cosmos.desired_angle = desired_angle;

		//motor.write(motor_power);

		last_update_time = udpate_time;

		IMU_updated = false;
	}
}

void update_IMU()
{

}
