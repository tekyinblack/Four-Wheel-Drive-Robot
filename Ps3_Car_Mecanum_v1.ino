/* PS3 controller program for an ESP32 D1 board, same layout as Arduino Uno
 * This program is for use with two L298D motor controllers driving four
 * mecanum wheels.
 * It's intended to provide forward, backward, spin left, spin right, 
 * move sideways left, move sideways right and move diagonally
*/
#include <Ps3Controller.h>
// change variables
int oldy = 0;
int oldx = 0;
int oldrx = 0;

// L298D driver pins
#define RightFrontEnable 26
#define RightFrontPin1 17
#define RightFrontPin2 25
#define RightRearEnable 18
#define RightRearPin1 23
#define RightRearPin2 19

#define LeftFrontEnable 14
#define LeftFrontPin1 27
#define LeftFrontPin2 16
#define LeftRearEnable 12
#define LeftRearPin1 13
#define LeftRearPin2 5

#define MAX_MOTOR_SPEED 200

int connectedLED = 0;
// PWM channels and info
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightFrontSpeedChannel = 4;
const int rightRearSpeedChannel = 5;
const int leftFrontSpeedChannel = 6;
const int leftRearSpeedChannel = 7;

// receive PS3 info and process
void notify() {

  int yAxis = -(Ps3.data.analog.stick.ly);  //Left stick  - y axis - forward/backward car movement
  int xAxis = (Ps3.data.analog.stick.lx);   //Left stick - x axis - left/right car movement
  int rxAxis = (Ps3.data.analog.stick.rx);  //Right stick - x axis - left/right car movement

  if (oldy == yAxis && oldx == xAxis && oldrx == rxAxis) {
    return;
  }  // if no change then nothing to process, may change this if time is set
  oldy = yAxis;
  oldx = xAxis;
  oldrx = rxAxis;

  yAxis = map(yAxis, -128, 127, -256, 255);
  xAxis = map(xAxis, -128, 127, -256, 255);
  rxAxis = map(rxAxis, -128, 127, -256, 255);

  double denominator = max(abs(yAxis) + abs(xAxis) + abs(rxAxis), 255);
  double frontLeftPower = ((yAxis + xAxis + rxAxis) / denominator) * MAX_MOTOR_SPEED;
  double RearLeftPower = ((yAxis - xAxis + rxAxis) / denominator) * MAX_MOTOR_SPEED;
  double frontRightPower = ((yAxis - xAxis - rxAxis) / denominator) * MAX_MOTOR_SPEED;
  double RearRightPower = ((yAxis + xAxis - rxAxis) / denominator) * MAX_MOTOR_SPEED;
  Serial.println("yAxis:" + String(yAxis) + " xAxix:" + String(xAxis) + " rxAxis:" + String(rxAxis));
  Serial.println("frontLeftPower:" + String(frontLeftPower) + " RearLeftPower:" + String(RearLeftPower) + " FrontRightPower:" + String(frontRightPower) + " RearRightPower:" + String(RearRightPower));

  rotateMotor(frontRightPower, RearRightPower, frontLeftPower, RearLeftPower);
}

// change the motor drives
void rotateMotor(int frontRightSpeed, int rearRightSpeed, int frontLeftSpeed, int rearLeftSpeed) {

  if (frontRightSpeed < 0) {
    digitalWrite(RightFrontPin1, LOW);
    digitalWrite(RightFrontPin2, HIGH);
  } else if (frontRightSpeed > 0) {
    digitalWrite(RightFrontPin1, HIGH);
    digitalWrite(RightFrontPin2, LOW);
  } else {
    digitalWrite(RightFrontPin1, LOW);
    digitalWrite(RightFrontPin2, LOW);
  }

  if (rearRightSpeed < 0) {
    digitalWrite(RightRearPin1, LOW);
    digitalWrite(RightRearPin2, HIGH);
  } else if (rearRightSpeed > 0) {
    digitalWrite(RightRearPin1, HIGH);
    digitalWrite(RightRearPin2, LOW);
  } else {
    digitalWrite(RightRearPin1, LOW);
    digitalWrite(RightRearPin2, LOW);
  }

  if (frontLeftSpeed < 0) {
    digitalWrite(LeftFrontPin1, LOW);
    digitalWrite(LeftFrontPin2, HIGH);
  } else if (frontLeftSpeed > 0) {
    digitalWrite(LeftFrontPin1, HIGH);
    digitalWrite(LeftFrontPin2, LOW);
  } else {
    digitalWrite(LeftFrontPin1, LOW);
    digitalWrite(LeftFrontPin2, LOW);
  }

  if (rearLeftSpeed < 0) {
    digitalWrite(LeftRearPin1, LOW);
    digitalWrite(LeftRearPin2, HIGH);
  } else if (rearLeftSpeed > 0) {
    digitalWrite(LeftRearPin1, HIGH);
    digitalWrite(LeftRearPin2, LOW);
  } else {
    digitalWrite(LeftRearPin1, LOW);
    digitalWrite(LeftRearPin2, LOW);
  }
  ledcWrite(rightFrontSpeedChannel, abs(frontRightSpeed));
  ledcWrite(rightRearSpeedChannel, abs(rearRightSpeed));

  ledcWrite(leftFrontSpeedChannel, abs(frontLeftSpeed));
  ledcWrite(leftRearSpeedChannel, abs(rearLeftSpeed));
}

// setup pins routine, separate from main setup routine
void setUpPinModes() {
  pinMode(RightFrontEnable, OUTPUT);
  pinMode(RightFrontPin1, OUTPUT);
  pinMode(RightFrontPin2, OUTPUT);

  pinMode(RightRearEnable, OUTPUT);
  pinMode(RightRearPin1, OUTPUT);
  pinMode(RightRearPin2, OUTPUT);

  pinMode(LeftFrontEnable, OUTPUT);
  pinMode(LeftFrontPin1, OUTPUT);
  pinMode(LeftFrontPin2, OUTPUT);

  pinMode(LeftRearEnable, OUTPUT);
  pinMode(LeftRearPin1, OUTPUT);
  pinMode(LeftRearPin2, OUTPUT);


  //Set up PWM for motor speed
  ledcSetup(rightFrontSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftFrontSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(rightRearSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftRearSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(RightFrontEnable, rightFrontSpeedChannel);
  ledcAttachPin(LeftFrontEnable, leftFrontSpeedChannel);
  ledcAttachPin(RightRearEnable, rightRearSpeedChannel);
  ledcAttachPin(LeftRearEnable, leftRearSpeedChannel);

  rotateMotor(0, 0, 0, 0);
}

// on PS3 connected, turn the LED on
void onConnect() {
  Serial.println("Connected.");
  connectedLED = 1;
}

// on PS3 disconnect, turn the LED off
void onDisConnect() {
  rotateMotor(0, 0, 0, 0);
  connectedLED = 0;
}

// main setup routine, call teh pin setup and inituialise PS3 connect
void setup() {
  setUpPinModes();
  Serial.begin(115200);
  Serial.println(String(__FILE__) + " " + String(__DATE__) + " " + String(__TIME__));

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("A0:5A:5A:A0:09:84");

  Serial.println("Ready.");
}

// main loop just flashes the LED 
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
