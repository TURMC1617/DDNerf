/*
  Simple code to drive the Nerf gun via serial
  
  Serial message is a null-terminated string
  with the first byte corresponding to the
  control mode of the gun (0 for PID mode, 1
  for continuous angle mode), the second byte
  corresponding to the relay state, and the
  rest of the string being the angle as a
  float.
  
  Based on
 http://www.arduino.cc/en/Tutorial/SerialEvent
 
 */
 
 #include <Servo.h>
 
const unsigned char servoPin = 9;
const unsigned char relayPin = 7;

Servo mainMotor;
String angleString = "";



String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  Serial.begin(9600);
  
  inputString.reserve(50);
  angleString.reserve(50);
  
  mainMotor.attach(servoPin);
  pinMode(relayPin, OUTPUT);
  
  Serial.println("0");
}

void loop() {
  if (stringComplete) {
    char controlByte = inputString.charAt(0);
    char relayByte = inputString.charAt(1);
    angleString = inputString.substring(2);
    
    if (controlByte == '1') {
      float servoAngle = atof(angleString.c_str());
      mainMotor.write(servoAngle);
    } else {
      // Do PID control
      mainMotor.write(90);
    }
    
    if (relayByte == '1') {
      digitalWrite(relayPin, HIGH);
    } else {
      digitalWrite(relayPin, LOW);
    }
    
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
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
      Serial.println("0");
    }
  }
}


