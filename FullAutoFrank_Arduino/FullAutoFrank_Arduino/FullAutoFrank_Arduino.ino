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

// Helper structs
typedef struct Rotation {
	float pitch, yaw, roll;
};

typedef struct MpuVector {
	int16_t x, y, z;
};

// Motor declaration
Servo motor;

// Serial Comms
String input_string = "";
String angle_string = "";
boolean string_complete = false;

// State Variables
boolean manual_mode = false;
float desired_angle = 0;

// MPU Variables
MPU6050 mpu;
Rotation r, v, a;
MpuVector accel, gyro;

// PID Variables
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

// The setup function runs once when you press reset or power the board
void setup() {
	r.pitch = 0;
	r.yaw = 0;
	r.roll = 0;
  
	Serial.begin(576000);

	input_string.reserve(200);
	angle_string.reserve(50);

	//Setup Motor
	motor.attach(MOTOR_PIN);


	// Start IMU
	mpu.initialize();
	pinMode(IMU_PIN1, OUTPUT);
	pinMode(IMU_PIN2, OUTPUT);
	pinMode(RELAY_PIN, OUTPUT);

	// MPU constants
	mpu.setXAccelOffset(-1812);
	mpu.setYAccelOffset(1379);
	mpu.setZAccelOffset(1427);
	mpu.setXGyroOffset(77);
	mpu.setYGyroOffset(-47);
	mpu.setZGyroOffset(64);

	// startupTone();
	Serial.println("0");
}

// the loop function runs over and over again until power down or reset
void loop() {
	check_serial();
	update_IMU();
    run_motors();
}

/*
SerialEvent occurs whenever a new data comes in the
hardware serial RX.  This routine is run between each
time loop() runs, so using delay inside loop can delay
response.  Multiple bytes of data may be available.
*/
void serial_event() {
	while (Serial.available()) {
		char in_char = (char)Serial.read();
		input_string += in_char;

		if (in_char == '\0') {
			string_complete = true;
		}
	}
}



void check_serial() {
	if (string_complete) {
		char control_byte = input_string.charAt(0);
		char relay_byte = input_string.charAt(1);
		angle_string = input_string.substring(2);

		// Check relay status
		if (relay_byte == '1') {
			digitalWrite(RELAY_PIN, HIGH);
		} else {
			digitalWrite(RELAY_PIN, LOW);
		}

		// Update serial parameters
		desired_angle = atof(angle_string.c_str());
		manual_mode = control_byte == '1';

		// Clear the string:
		input_string = "";
		string_complete = false;

		// Handshake and send latest IMU yaw
		Serial.println(r.yaw);
	}
}

void update_IMU() {
	mpu.getMotion6(&accel.x, &accel.y, &accel.z, &gyro.x, &gyro.y, &gyro.z);

	v.pitch = (gyro.x / 131);
	if (v.pitch == -1) { //error filtering
		v.pitch = 0;
	}

	v.roll = (gyro.y / 131);
	if (v.roll == 1) { //error filtering
		v.roll = 0;
	}

	v.yaw = gyro.z / 131;
	a.pitch = (v.pitch*0.046);
	a.roll = (v.roll*0.046);
	a.yaw = (v.yaw*0.045);

	r.pitch += a.pitch;
	r.roll += a.roll;
	r.yaw += a.yaw;
}

void run_motors() {
	if (manual_mode) {
		motor.write(desired_angle);
	}
	else {
		Desired_Setpoint = desired_angle;
		Measured_Input = r.yaw;
		pid_controller.Compute();
		motor.write(Motor_Output);
    }
}
