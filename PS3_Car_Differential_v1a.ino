/* Code to be used with an ESP32 D1 controller board, similar layout to an Arduino Uno
 * alongside a single L298D motor driver board. The robot has four wheels and a motor
 * for each wheel, the L298D is connected to and driving both wheels on a side in parallel.
 * This code has a degree of sophistication allowing proportional steering with both sets 
 * of wheels varying their speed to accomplish turning
*/
#include <Ps3Controller.h>

//Right motor pin definitions
int enableRightMotor = 14;
int rightMotorPin1 = 16;
int rightMotorPin2 = 27;
//Left motor pin definitions
int enableLeftMotor = 26;
int leftMotorPin1 = 25;
int leftMotorPin2 = 17;

#define MAX_MOTOR_SPEED 255
#define THRESHOLD 25

#define CHASSIS_WIDTH 6.5  // wheel to wheel distance / 2

long oldLeftValue = 0;
long oldRightValue = 0;

int connectedLED = 0;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

// vary the power applied to prevent on/off working of the motors
long applyCurve(long joystick) {
  if (joystick > 0) {
    return (MAX_MOTOR_SPEED - THRESHOLD) * map(pow(joystick, 3), -2097152, 2048383, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) / MAX_MOTOR_SPEED + THRESHOLD;
  } else if (joystick < 0) {
    return (MAX_MOTOR_SPEED - THRESHOLD) * map(pow(joystick, 3), -2097152, 2048383, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) / MAX_MOTOR_SPEED - THRESHOLD;
  }
  return 0;
}

// accomodate PS3 controllers with imperfect rest positions
long deadZone(long joystick) {
  if (abs(joystick) < THRESHOLD) {
    return 0;
  }
  return joystick;
}

// handle teh input from the PS3 controllers
void notify() {
  long leftValue = (Ps3.data.analog.stick.ly);   //Left stick  - y axis - forward/backward car movement
  long rightValue = (Ps3.data.analog.stick.rx);  //Right stick - x axis - left/right car movement
  long Vl, Vr;
  if (oldLeftValue == leftValue && oldRightValue == rightValue) {
    return;
  }
  oldLeftValue = leftValue;
  oldRightValue = rightValue;

  leftValue = deadZone(leftValue);
  //rightValue = deadZone(rightValue);

  leftValue = applyCurve(leftValue);
  rightValue = applyCurve(rightValue);

  // rightValue = map(rightValue, -128, 127, -256, 255);
  long radius = 256 - abs(rightValue);
  double factor = (radius - CHASSIS_WIDTH) / (radius + CHASSIS_WIDTH);


  Serial.println("left:" + String(leftValue) + " right:" + String(rightValue));

  if (rightValue == 0)  // first quadrant
  {
    Vl = leftValue;
    Vr = leftValue;
  } else if (rightValue >= 0 && leftValue <= 0)  // first quadrant
  {
    Vl = leftValue;
    Vr = leftValue * factor;
  } else if (rightValue < 0 && leftValue <= 0)  // second quadrant
  {
    Vr = leftValue;
    Vl = leftValue * factor;
  } else if (rightValue >= 0 && leftValue > 0)  // fourth quadrant
  {
    Vl = leftValue;
    Vr = leftValue * factor;
  } else if (rightValue < 0 && leftValue > 0)  // third quadrant
  {
    Vr = leftValue;
    Vl = leftValue * factor;
  } else {
    Vl = 0;
    Vr = 0;
  }
  Serial.println("Vl:" + String(Vl) + " Vr:" + String(Vr));

  rotateMotor(Vr, Vl);
}

// on PS3 connected, light LED
void onConnect() {
  Serial.println("Connected!.");
  connectedLED = 1;
}
// on PS3 disconnected, extinguish LED
void onDisConnect() {
  rotateMotor(0, 0);
  connectedLED = 0;
}

// apply motor power
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }
  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
}

// setup routine for pins
void setUpPinModes() {
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  //Set up PWM for motor speed
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

  rotateMotor(0, 0);
}

//  Main setup routine, contains the PS3 address which must be updated
void setup() {
  setUpPinModes();
  Serial.begin(115200);
  Serial.println(String(__FILE__) + " " + String(__DATE__) + " " + String(__TIME__));

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisConnect);
  // Ps3.begin();
  Ps3.begin("A0:5A:5A:A0:09:84");  // <---- Update this address for the PS3 in use
  Serial.println("Ready.");
}

// main loop flashes LED to indicate PS3 connection
void loop() {
  if (connectedLED) {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}