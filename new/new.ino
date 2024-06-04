/* OBR 2024 Code - Nióbio */
/* Made by Moisés Corrêa Gomes */

/* Libraries section */
#include "Adafruit_TCS34725softi2c.h"
#include <MPU6050_light.h>
#include "I2Cdev.h"
#include "Wire.h"
#include <Arduino_JSON.h>

/* General configurations section */
bool developerMode = true;
bool controllerMode = false;
bool run = false;
int motorLVelocity = 230;
int motorRVelocity = 230;
#define colorVelocity TCS34725_INTEGRATIONTIME_50MS
#define colorGain TCS34725_GAIN_4X
uint8_t irMultiplicationFactors[5] = {2, 20, 20, 20, 2};
int irBorder = 20400;
int cycle = 1;

/* Pin declaration and variables section */
// IR Sensor pins
uint8_t irSensorPins[5] = {A0, A1, A2, A3, A4};
// Ultrassonic Pins {trig, echo, ...}
uint8_t ultraPins[6] = {35, 34, 37, 36, 39, 38};
// Grouped Stepper Motor pins {pwm min, a pin, b pin, ...}
uint8_t motorPins[6] = {8, 9, 10, 13, 11, 12};
// TCS34725 Pins {sda, scl, ...}
uint8_t colorPins[4] = {31, 30, 33, 32};

/* Objects declaration*/
// TCS34725
Adafruit_TCS34725softi2c rightColor = Adafruit_TCS34725softi2c(colorVelocity, colorGain, colorPins[0], colorPins[1]);
Adafruit_TCS34725softi2c leftColor = Adafruit_TCS34725softi2c(colorVelocity, colorGain, colorPins[2], colorPins[3]);
// MPU6050
MPU6050 mpu(Wire);
// Sensor Values
JSONVar sensorData;

/* MPU6050 configuration */
bool isMpuUpsideDown = true;

/* Function declaration section */
int irSensorRead(int irNumber);
float ultraSensorRead(int trigPin, int echoPin);
uint16_t colorSensorRead(char side, char color);
float mpuSensorRead(char axis);
void motorControl(char side, char direction);
bool pinsSetup();
bool colorSetup();
bool mpuSetup();
void calibrateMpu(int speed, int loops);
void sensorsDebug();
void movementDebug();
void updateCycle(int number);

/* Main code section */
void setup()
{
  if (developerMode || controllerMode)
  {
    Serial.begin(115200);
    while (!Serial)
      ;
  }
  pinsSetup();
  colorSetup();
  mpuSetup();
  //calibrateMpu(1, 1000);
}

void loop()
{
  // movementDebug
  // ☖ ☗ ☖
  if (irSensorRead(3) < irBorder && irSensorRead(2) > irBorder && irSensorRead(4) > irBorder) {
    motorControl('B', 'F');
  } 
  // ☗ ☗ ☖ , ☗ ☖ ☖
  else if (irSensorRead(4) > irBorder && ((irSensorRead(3) < irBorder && irSensorRead(2) < irBorder) || (irSensorRead(3) > irBorder && irSensorRead(2) < irBorder))) {
    motorControl('B', 'L');
  } 
  // ☖ ☗ ☗ , ☖ ☖ ☗
  else if (irSensorRead(2) > irBorder && ((irSensorRead(3) < irBorder && irSensorRead(4) < irBorder) || (irSensorRead(3) > irBorder && irSensorRead(4) < irBorder))) {
    motorControl('B', 'R');
  }

  if (cycle == 10) updateCycle(-1)
  else {
    sensorsDebug();
    updateCycle(0);
  }
}

int irSensorRead(int irNumber)
{
  return analogRead(irSensorPins[irNumber - 1]) * irMultiplicationFactors[irNumber - 1];
}

float ultraSensorRead(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.0343 / 2;
}

uint16_t colorSensorRead(char side, char color)
{
  uint16_t r, g, b, c;
  switch (side)
  {
  case 'L':
    leftColor.getRawData(&r, &g, &b, &c);
    switch (color)
    {
    case 'R':
      return r;
      break;
    case 'G':
      return g;
      break;
    case 'B':
      return b;
      break;
    }
    break;
  case 'R':
    rightColor.getRawData(&r, &g, &b, &c);
    switch (color)
    {
    case 'R':
      return r;
      break;
    case 'G':
      return g;
      break;
    case 'B':
      return b;
      break;
    }
    break;
  }
}

float mpuSensorRead(char axis)
{
  mpu.update();
  switch (axis)
  {
  case 'X':
    return mpu.getAngleX();
    break;
  case 'Y':
    return mpu.getAngleY();
    break;
  case 'Z':
    return mpu.getAngleZ();
    break;
  }
}

void motorControl(char side, char direction)
{
  if (side == 'L')
  {
    switch (direction)
    {
    case 'F':
      analogWrite(motorPins[0], motorLVelocity);
      digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], LOW);
      break;
    case 'B':
      analogWrite(motorPins[0], motorLVelocity);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], HIGH);
      break;
    case 'S':
      analogWrite(motorPins[0], 0);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], LOW);
      break;
    }
  }
  else if (side == 'R')
  {
    switch (direction)
    {
    case 'F':
      analogWrite(motorPins[3], motorRVelocity);
      digitalWrite(motorPins[4], HIGH);
      digitalWrite(motorPins[5], LOW);
      break;
    case 'B':
      analogWrite(motorPins[3], motorRVelocity);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], HIGH);
      break;
    case 'S':
      analogWrite(motorPins[3], 0);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], LOW);
      break;
    }
  }
  else if (side == 'B')
  {
    switch (direction)
    {
    case 'F':
      analogWrite(motorPins[0], motorLVelocity);
      analogWrite(motorPins[3], motorRVelocity);
      digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], LOW);
      digitalWrite(motorPins[4], HIGH);
      digitalWrite(motorPins[5], LOW);
      break;
    case 'B':
      analogWrite(motorPins[0], motorLVelocity);
      analogWrite(motorPins[3], motorRVelocity);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], HIGH);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], HIGH);
      break;
    case 'L':
      analogWrite(motorPins[0], motorLVelocity);
      analogWrite(motorPins[3], motorRVelocity);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], HIGH);
      digitalWrite(motorPins[4], HIGH);
      digitalWrite(motorPins[5], LOW);
      break;
    case 'R':
      analogWrite(motorPins[0], motorLVelocity);
      analogWrite(motorPins[3], motorRVelocity);
      digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], LOW);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], HIGH);
      break;
    case 'S':
      analogWrite(motorPins[0], 0);
      analogWrite(motorPins[3], 0);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], LOW);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], LOW);
      break;
    }
  }
}

bool pinsSetup()
{
  for (int i = 0; i < 5; i++)
  {
    pinMode(irSensorPins[i], INPUT);
  }
  for (int i = 0; i < 6; i++)
  {
    pinMode(motorPins[i], OUTPUT);
  }
  for (int i = 0; i < 6; i = i + 2)
  {
    pinMode(ultraPins[i], OUTPUT);
    pinMode(ultraPins[i + 1], INPUT);
  }
  if (developerMode)
    Serial.println(F("Pins configured sucessfully"));
  return true;
}

bool colorSetup()
{
  bool leftColorStatus = leftColor.begin() ? true : false;
  bool rightColorStatus = rightColor.begin() ? true : false;
  if (leftColorStatus)
    if (developerMode) Serial.println(F("Left TCS34725 connected sucessfully"));
  else
  {
    if (developerMode) Serial.println(F("Left TCS34725 connection failed"));
  }
  if (rightColorStatus)
    if (developerMode) Serial.println(F("Right TCS34725 connected sucessfully"));
  else
  {
    if (developerMode) Serial.println(F("Right TCS34725 connection failed"));
  }
  if (leftColorStatus || rightColorStatus)
    return true;
  else
    return false;
}

bool mpuSetup()
{
  Wire.begin();
  byte status = mpu.begin();
  if (status != 0)
  {
    if (developerMode)
      Serial.println(F("MPU6050 connection failed"));
    return false;
  }
  if (developerMode)
    Serial.println(F("MPU6050 connected sucessfully"));
  mpu.upsideDownMounting = isMpuUpsideDown;
  mpu.calcOffsets();
  if (developerMode)
    Serial.println(F("MPU6050 calculated offsets"));
  return true;
}

void calibrateMpu(int speed, int loops)
{
  if (developerMode)
    Serial.println(F("Starting MPU6050 calibration"));
  for (int i = 0; i < loops; i++)
  {
    if (developerMode)
    {
      Serial.print(F("Calibration loop: "));
      Serial.println(i);
    }
    mpuSensorRead('X');
    mpuSensorRead('Y');
    mpuSensorRead('Z');
    delay(speed);
  }
  if (developerMode)
    Serial.println(F("Finished MPU6050 calibration"));
}

void sensorsDebug()
{
  sensorData["mpuX"] = (float) mpuSensorRead('X');
  sensorData["mpuY"] = (float) mpuSensorRead('Y');
  sensorData["mpuZ"] = (float) mpuSensorRead('Z');

  sensorData["leftColorR"] = (uint16_t) colorSensorRead('L', 'R');
  sensorData["leftColorG"] = (uint16_t) colorSensorRead('L', 'G');
  sensorData["leftColorB"] = (uint16_t) colorSensorRead('L', 'B');

  sensorData["rightColorR"] = (uint16_t) colorSensorRead('R', 'R');
  sensorData["rightColorG"] = (uint16_t) colorSensorRead('R', 'G');
  sensorData["rightColorB"] = (uint16_t) colorSensorRead('R', 'B');

  sensorData["leftUltra"] = (float) ultraSensorRead(ultraPins[0], ultraPins[1]);
  sensorData["centerUltra"] = (float) ultraSensorRead(ultraPins[2], ultraPins[3]);
  sensorData["rightUltra"] = (float) ultraSensorRead(ultraPins[4], ultraPins[5]);

  // sensorData["ir1"] = (int) irSensorRead(1);
  sensorData["ir2"] = (int) irSensorRead(2);
  sensorData["ir3"] = (int) irSensorRead(3);
  sensorData["ir4"] = (int) irSensorRead(4);
  // sensorData["ir5"] = (int) irSensorRead(5);

  String sensorDataString = JSON.stringify(sensorData);

  Serial.println(sensorDataString);
}

void movementDebug()
{
  Serial.print("Waiting for command: ");
  while (true) {
    if (Serial.available() > 0)
    {
      char incomingChar = Serial.read();
      switch (incomingChar)
      {
      case 'F':
        motorControl('B', 'F');
        break;
      case 'B':
        motorControl('B', 'B');
        break;
      case 'L':
        motorControl('B', 'L');
        break;
      case 'R':
        motorControl('B', 'R');
        break;
      case 'S':
        motorControl('B', 'S');
        break;
      }
    }
    delay(1);
  }
}

void updateCycle(int number) {
  if (number == -1) cycle = 1;
  else cycle += 1;
}