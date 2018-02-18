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
#define IMU_PIN1 3
#define IMU_PIN2 5
#define BUZZER_PIN 4

Servo motor;

// Serial Comms
String inputString = "";
String angleString = "";
boolean stringComplete = false;


//float desired_angle;

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




//PID Variables
float kp = 2.5;
float ki = 4.0;
float kd = 25.0;

double Desired_Setpoint;
double Measured_Input;
double Motor_Output;

PID pid_controller(&Measured_Input, &Motor_Output, &Desired_Setpoint, kp, ki, kd, P_ON_M, DIRECT);
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



void startupTone() {
	tone(BUZZER_PIN, 262);
	delay(1000);
	tone(BUZZER_PIN, 330);
	delay(1000);
	tone(BUZZER_PIN, 392);
	delay(1000);
	tone(BUZZER_PIN, 523);
	delay(1000);
	noTone(BUZZER_PIN);

}

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(576000);

	inputString.reserve(200);
	angleString.reserve(50);

	//Setup Motor
	motor.attach(MOTOR_PIN);


	// Start IMU
	mpu.initialize();
	pinMode(IMU_PIN1, OUTPUT);
	pinMode(IMU_PIN2, OUTPUT);
	pinMode(RELAY_PIN, OUTPUT);
	mpu.setXAccelOffset(-1812);
	mpu.setYAccelOffset(1379);
	mpu.setZAccelOffset(1427);
	mpu.setXGyroOffset(77);
	mpu.setYGyroOffset(-47);
	mpu.setZGyroOffset(64);

	startupTone();
	Serial.println("0");

}

// the loop function runs over and over again until power down or reset
void loop() {

	check_serial();
}

/*
SerialEvent occurs whenever a new data comes in the
hardware serial RX.  This routine is run between each
time loop() runs, so using delay inside loop can delay
response.  Multiple bytes of data may be available.
*/
void serialEvent() {
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		// add it to the inputString:
		inputString += inChar;
		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\0') {
			stringComplete = true;
		}
	}
}



void check_serial()
{
	if (stringComplete)
	{
		char controlByte = inputString.charAt(0);
		char relayByte = inputString.charAt(1);
		angleString = inputString.substring(2);

		if (controlByte == '1')
		{
			float desired_angle = atof(angleString.c_str());
			motor.write(desired_angle);
		}
		// Do PID Control for autonomous
		else if (controlByte = '2')
		{
			float desired_angle = atof(angleString.c_str());
			run_motors(desired_angle);

		}

		// Check relay status
		if (relayByte == '1')
		{
			digitalWrite(RELAY_PIN, HIGH);
		}
		else
		{
			digitalWrite(RELAY_PIN, LOW);
		}

		//clear the string:
		inputString = "";
		stringComplete = false;
	}
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

void run_motors(float desired_angle)
{
	
	update_IMU();
	Serial.print("yaw: ");
	Serial.println(yaw);
	
	if (IMU_updated)
	{
		Desired_Setpoint = desired_angle;
		Measured_Input = yaw;
		pid_controller.Compute();
		motor.write(Motor_Output);
		IMU_updated = false;
	}


}
