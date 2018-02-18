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

Servo motor;



float desired_angle;

//IMU global variables
bool IMU_updated = false;

bool power_off = true;



struct imuStruct {
	float yaw;
	float pitch;
	float roll;

} IMU, IMU_init, IMU_adjusted;





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

	motor.attach(MOTOR_PIN);
	
	//pinMode(MOTOR_PIN, OUTPUT);

	Serial.begin(576000);

	//pinMode(MOTOR_PIN, OUTPUT);
	pinMode(RELAY_PIN, OUTPUT);


	IMU.yaw = 0.0;
	IMU.pitch = 0.0;
	IMU.roll = 0.0;



}

// the loop function runs over and over again until power down or reset
void loop() {


	//check_serial();
	//update_IMU();
	//run_motors();
  
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
