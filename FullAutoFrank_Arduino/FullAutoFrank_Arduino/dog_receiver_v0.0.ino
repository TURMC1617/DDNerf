/*
   pumpkinbot_remote_v0.0.ino
   receives commands from remote to drive motors
   date:  10/25/2016
   Author: William Osman (william.m.osman@gmail.com)
*/

#include <SPI.h>
#include "RF24.h"


struct dataStruct {
  bool enable;
  float angle;
} command;

float kp = 2.5;
float ki = 4.0;
float kd = 25.0;

//IMU GLOBALs
bool synched = false;
bool IMU_updated = false;
bool update_initAngle  = true; //update intial position flag

HardwareSerial *Serial_IMU = &Serial1;//&Serial;


struct imuStruct {
  float yaw;
  float pitch;
  float roll;
} IMU, IMU_init, IMU_adjusted;

bool watchdog_flag = false;

float error_test = 0.0;

bool limit_cw, limit_ccw;

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

void setup() {
  Serial.begin(57600);
  uint32_t timeout = 2000;
  uint32_t start = millis();
  while(!Serial  && (millis() - start < timeout));

  Serial_IMU->begin(57600);

  radio_config();

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(MOTOR_DIR_1, OUTPUT);
  pinMode(MOTOR_DIR_2, OUTPUT);
  pinMode(LIMIT_CW , INPUT_PULLUP);
  pinMode(LIMIT_CCW, INPUT_PULLUP);

  delay(1000);
  //Serial.println("begin");

  IMU_connect();

  command.enable = false;
  command.angle = 0.0;

  IMU.yaw = 0.0;
  IMU.pitch = 0.0;
  IMU.roll = 0.0;

  cosmos.id = 1;
  cosmos.desired_angle = 0.0;
  cosmos.current_angle = 1.0;
  cosmos.motor_power = 99;
}

void loop()
{
  check_radio();
  check_IMU();
  run_motors();

  static uint32_t timer = millis();

  if (millis() - timer > 50) {
    /*
    Serial.print("yaw: "), Serial.print(IMU.yaw);
    Serial.print(" adjusted: "), Serial.print(IMU_adjusted.yaw);
    Serial.print(" error: "), Serial.print(error_test);
    Serial.println();
    */
    #ifdef COSMOS
    sendTlm();
    #endif
    timer = millis();
  }
}



//************** MOTOR CONTROL *****************************************

void run_motors()
{
  static uint32_t last_update_time = millis();

  uint32_t update_time = millis();

  //turn off all motors
  if ( watchdog_flag ) {
    set_motor( MOTOR_PIN, 0, MOTOR_DIR_1, MOTOR_DIR_2 );
    //Serial.println("motors off");
  }

  //update motor control loop
  else if ( IMU_updated ) {
    //update motor speeds

    float error = IMU_adjusted.yaw - command.angle;
    float motor_power = getPID(error);
    error_test = error;

    cosmos.motor_power = motor_power;
    cosmos.current_angle = IMU_adjusted.yaw;
    cosmos.desired_angle = command.angle;

    set_motor( MOTOR_PIN, -motor_power, MOTOR_DIR_1, MOTOR_DIR_2 );

    last_update_time = update_time;

    IMU_updated = false;
    /*
      Serial.print(pumpkin_bot.left_wheel);
      Serial.print(", ");
      Serial.print(pumpkin_bot.right_wheel);
      Serial.print(", ");
      Serial.println(pumpkin_bot.pumpkin);
    */
  }

}

void set_motor(byte pin, int16_t power, byte dir_1, byte dir_2)
{
  int adjusted_power = constrain(power, -255, 255);
  uint8_t pwm_write = constrain(abs(adjusted_power), 0, MAX_PWM);
  
  //"forward"
  if (adjusted_power > 0 && !limit_cw) {
    digitalWrite(dir_1, HIGH);
    digitalWrite(dir_2, LOW);
    analogWrite(pin, pwm_write);
  }
  //"reverse"
  else if (adjusted_power < 0 && !limit_ccw) {
    digitalWrite(dir_1, LOW);
    digitalWrite(dir_2, HIGH);
    analogWrite(pin, pwm_write);
  }
  //break
  else {
    digitalWrite(dir_1, LOW);
    digitalWrite(dir_2, LOW);
    analogWrite(pin, 255);
  }
}

#define I_FADE  0.05  //time constant sorta setup for I rolling average
float getPID(float error)
{
 
  static float last_error = 0;
  static float last_i = 0;

  float p = error;
  float i = (error * I_FADE) + (last_i * (1.0 - I_FADE) );
  float d = (error - last_error);

  if(abs(d) > 3.0) last_i = 0;

  cosmos.p = p;
  cosmos.i = i;
  cosmos.d = d;

  float pid = (kp * p) + (ki * i) + (kd * d);

  last_error = error;
  last_i = i;

  return pid;
}

//************** NRF24 RADIO *****************************************
void radio_config()
{
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(sizeof(command));
  //radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setDataRate(RF24_250KBPS);


  radio.setChannel(53);
  //radio.setRetries(2, 5);
  //radio.setAutoAck(1);
  //Serial.println(radio.getChannel());

  radio.startListening();
}

void check_radio()
{
  static uint32_t last_update_time = millis();

  if ( radio.available() ) {
    while ( radio.available() ) {
      radio.read( &command, sizeof(command) );
    }
    //Serial.println("receive");
    if ( radio.txStandBy(10) ) last_update_time = millis();
    //if ( !radio.txStandBy(10) ) Serial.println("fail");
  }

  #ifdef WATCHDOG
  if (millis() - last_update_time >= RADIO_WATCHDOG_TIME) watchdog_flag = true;
  else watchdog_flag = false;
  #endif
}

//************** IMU SPARKFUN RAZOR *****************************************
#define ROLL_AVG 0.9 //rolling average

void check_IMU() {
  while (Serial_IMU->available() > 12) {
    //Serial.println(Serial_IMU->available());
    static imuStruct IMU_raw;

    static int16_t rollover_yaw = 0;
    static int16_t rollover_roll = 0;

    IMU_raw.yaw = readYPR();
    IMU_raw.pitch = readYPR();
    IMU_raw.roll = readYPR();

    IMU = IMU_raw;

    static imuStruct IMU_old = IMU_raw;

    rollover_yaw = handle_rollover(IMU_raw.yaw, IMU_old.yaw, rollover_yaw);
    rollover_roll = handle_rollover(IMU_raw.roll, IMU_old.roll, rollover_roll);

    IMU_old = IMU_raw;

    //Serial.println(IMU_raw.yaw);


    if (update_initAngle) {
      IMU_init.yaw = IMU_raw.yaw + (rollover_yaw * 360.0) - IMU_adjusted.yaw;
      //IMU_init.pitch = IMU_raw.pitch - IMU_adjusted.pitch;
      //IMU_init.roll = IMU_raw.roll + (rollover_roll * 360.0) - IMU_adjusted.roll;
      update_initAngle = false;
    }

    //convert to absolute angle, relative to initial angle
    else {
      IMU_adjusted.yaw = IMU_raw.yaw - IMU_init.yaw + (rollover_yaw * 360.0);
      IMU_adjusted.pitch = IMU_raw.pitch - IMU_init.pitch;
      IMU_adjusted.roll = IMU_raw.roll - IMU_init.roll + (rollover_roll * 360.0);
    }


    IMU_updated = true;
  }
}

float rolling_average(float data, float new_data, float amount) {
  return (data * amount) + (new_data * (1.0 - amount));
}

//returns true if both numbers are the same sign
bool SameSign(float x, float y) {
  return (x >= 0.0) ^ (y < 0.0);
}

bool Positive(float x) {
  return (x >= 0.0);
}

int16_t handle_rollover(float data, float old_data, int16_t rollover) {
  //IMU outputs -180° to 180° degrees
  //find rollover by checking for a sign change
  //ignore 0° crossover
  if ( !SameSign(data, old_data) ) {

    //indicates positive rollover
    if ( data < -90.0 ) rollover++;

    //indicates negative rollover
    else if ( data > 90.0 ) rollover--;
  }

  return rollover;
}

void IMU_connect() {
  Serial_IMU->write("#ob");  // Turn on binary output
  Serial_IMU->write("#o0");  // Turn off continuous streaming output
  Serial_IMU->write("#oe0"); // Disable error message output

  // Synch with Razor
  Serial_IMU->flush();  // Clear input buffer up to here
  Serial_IMU->write("#s00");  // Request synch token
  waitForSync();
  Serial_IMU->flush();
  Serial_IMU->write("#o1");


}

void waitForSync() {
  const uint16_t timeout = 2000;
  uint32_t start_time = millis();
  while (!synched && (millis() - start_time < timeout) )
  {
    synched = readToken("#SYNCH00\r\n");
  }
  if (!synched) Serial.println("IMU Sync Failed");
  // else debug_Serial_IMU.println("IMU Sync");
}

boolean readToken(String token) {
  // Wait until enough bytes are available
  if (Serial_IMU->available() < token.length()) return false;

  // Check if incoming bytes match token
  for (int i = 0; i < token.length(); i++)
  {
    if (Serial_IMU->read() != token.charAt(i)) return false;
  }

  return true;
}

//converts byte stream into 3 float values
//this whole setup is super ghetto and will get #rekt if the serial buffer overflows
float readYPR() {
  uint32_t tempBIN = 0;
  uint32_t FloatBIN = 0;

  tempBIN = (tempBIN | Serial_IMU->read());
  FloatBIN = FloatBIN | tempBIN;

  tempBIN = 0;
  tempBIN = (tempBIN | Serial_IMU->read()) << 8;
  FloatBIN = FloatBIN | tempBIN;

  tempBIN = 0;
  tempBIN = (tempBIN | Serial_IMU->read()) << 16;
  FloatBIN = FloatBIN | tempBIN;

  tempBIN = 0;
  tempBIN = (tempBIN | Serial_IMU->read()) << 24;
  FloatBIN = FloatBIN | tempBIN;


  float tempYPR = 0;
  *((uint32_t*)&tempYPR) = FloatBIN; //convert uint23_t to float

  return tempYPR;
}

//************** COSMOS TELEMETRY *****************************************
/*
void readCmds()
{
  
  if (Serial.available())
  {
    byte incoming_ID = Serial.read(); //load the first ID byte
    
    uint8_t* data; //address to store data
    byte data_length; //size of data to read

    switch(incoming_ID)
    {
      case INCOMING_ID_SETTING:
        data = (uint8_t*)&gsIncoming_setting;
        data_length = sizeof(gsIncoming_setting);
        if(!readTlm(data++, data_length)) break;

        carData.sample_rate = gsIncoming_setting.sample_rate;
        sample_delay = 1000 / gsIncoming_setting.sample_rate;
        break;
        
      case INCOMING_ID_EXECUTE:
        data = (uint8_t*)&gsIncoming_execute;
        data_length = sizeof(gsIncoming_execute);
        if(!readTlm(data++, data_length)) break;
        
        executeCmd(gsIncoming_execute.command);
        break;
        
      default:
        //invalid packet ID
        break;  
    }
  }
}

void executeCmd(byte command){
  switch(command){
    case CMD_NONE:
      //blank command
      digitalWrite(13, HIGH);
      break;
    case CMD_TEST:
      //test command
      digitalWrite(13, LOW);
      break;
    case CMD_SEND:
      //start sending data
      send_data = true;
      break;
    case CMD_STOP:
      //stop sending data
      send_data = false;
      break;
    default:
      //not recognized
      break;
    
  };
}

bool wait_4_Serial(uint16_t timeout_ms) //return 0 for timeout
{
  uint32_t start_time = millis();
  uint16_t time = 0;
  while(!Serial.available())
  {
    time = millis() - timeout_ms;
    if(time >= timeout_ms) return 0;
  }
  return 1;
}

bool readTlm(uint8_t* data, byte size) //return 0 for timeout
{
  for(int i = 0; i < size; i++)
    {
      if (!wait_4_Serial(10)) return 0;
      *data = Serial.read();
      data++;
    }
  return 1;
}

*/

void sendTlm()
{
  if(true)//if(send_data)
  {
    uint32_t timer = millis();
    
    cosmos.length = sizeof(cosmos);
    writeTlm((const char*)&cosmos, sizeof(cosmos));
    
    //carData.test = millis() - timer;
  }
}

void writeTlm(const char* pkt, byte size)
{
  for(int i=0; i<size; i++)
  {
    Serial.write(pkt[i]);
  }
}

