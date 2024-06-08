/* OBR 2024 Code - Nióbio */
/* Made by Moisés Corrêa Gomes */

/* Libraries section */
#include "Adafruit_TCS34725softi2c.h"
#include <MPU6050_light.h>
#include "I2Cdev.h"
#include "Wire.h"
#include <Arduino_JSON.h>

/* General configurations section */
// Modes
bool developerMode = true;
bool controllerMode = false;
// Color sensors configuration
#define colorVelocity TCS34725_INTEGRATIONTIME_2_4MS
#define colorGain TCS34725_GAIN_60X
// Gyroscope configuration
bool isGyroscopeMountedUpsideDown = true;
// Motor velocities
// 200 - 212 | 175 - 205 | 165 - 200
uint8_t motorLVelocity = 145 + 20;
uint8_t motorRVelocity = 180 + 20;
// Velocities used on smooth curves
uint8_t adjustmentLVelocity = 120 + 20;
uint8_t adjustmentRVelocity = 130 + 20;
uint8_t fastAdjustmentLVelocity = 200 + 20;
uint8_t fastAdjustmentRVelocity = 225 + 20;
// Ir sensors multiplication factor
uint8_t irMultiplicationFactors[5] = {20, 20, 20, 20, 20};
// Number between black and white ir readings to detect line
uint16_t irBorder = 20000;
// Cycle variable to keep track of things
uint16_t cycle = 1;
// Define if leds should be used to debug sensors
bool useLeds = true;

/* Pin declaration and variables section */
// IR Sensor pins
uint8_t irSensorPins[5] = {A3, A4, A5, A6, A7};
// Led pins
uint8_t ledPins[5] = {22, 24, 26, 28, 30};
// Ultrassonic Pins
uint8_t trigPins[3] = {48, 50, 52};
uint8_t echoPins[3] = {49, 51, 53};
// Grouped Stepper Motor pins {pwm min, a pin, b pin, ...}
uint8_t motorPins[6] = {8, 9, 10, 13, 11, 12};
// TCS34725 Pins {sda, scl, ...}
uint8_t colorPins[4] = {44, 45, 46, 47};
/* Objects declaration*/
// Color sensors (TCS34725)
Adafruit_TCS34725softi2c leftColor = Adafruit_TCS34725softi2c(colorVelocity, colorGain, colorPins[0], colorPins[1]);
Adafruit_TCS34725softi2c rightColor = Adafruit_TCS34725softi2c(colorVelocity, colorGain, colorPins[2], colorPins[3]);
// Gyroscope (MPU6050)
MPU6050 mpu(Wire);
// Sensor values
JSONVar sensorData;

/* Function declaration section */
// Returns analogRead of ir sensor. 1: far left, 2: left, 3: center, 4: right, 4: far right
uint16_t irSensorRead(int irNumber);
// Returns an array of boolean with all ir readings
bool * getIrReadings();
// Returns a boolean of just one ir
bool getIrReading(int irNumber);
// Returns distance of ultrassonic sensor. 1: left, 2: center, 3: right
float ultraSensorRead(int ultraNumber);
// Returns the desired color of one side. L: left or R: right, and R: red, G: green or B: blue
uint16_t colorSensorRead(char side, char color);
// Returns an angle of the gyroscope. X, Y and Z
float mpuSensorRead(char axis);
// Moves the motor to a side, L: left, B: both or R: right, and F: forward, B: backward, R: right, L: left or S: stop (L and R just works for B mode)
void motorControl(char side, char direction, uint8_t speedL, uint8_t speedR);
// Setups the arduino pins
bool pinsSetup();
// Setups the color sensor
bool colorSetup();
// Setups the gyroscope
bool mpuSetup();
// Calibrates the gyroscope giving it time to reach home position, receives a delay between readings and the quantity of readings
void calibrateMpu(int speed, int loops);
// Prints the sensors reading on the serial
void sensorsDebug();
// Waits for movement commands on the serial
void movementDebug();
// Updates a variable increasing it by one, if it receives -1, the cycle goes back to 1
void updateCycle(int number);
// Compares arrays
bool compareIrCase(bool irReadings[], bool irCase[]);
// Checks ir cases
uint8_t checkIrCase();
// motorMovement automations
void forwardAll();
void forwardLeft();
void forwardRight();
void backwardAll();
void backwardLeft();
void backwardRight();
void stopAll();
void stopRight();
void stopLeft();
void leftStraight();
void rightStraight();
void leftAdjust();
void rightAdjust();
void leftFastAdjust();
void rightFastAdjust();

/* Main code section */
void setup()
{
  // Waits for serial monitor to be available
  if (developerMode || controllerMode)
  {
    Serial.begin(115200);
    while (!Serial)
      ;
  }
  pinsSetup();
  //colorSetup(); 
  //mpuSetup();
  // calibrateMpu(1, 500);
}

void loop()
{
  int straightMoveDelay = 150;
  int straightStopDelay = 40;
  stopAll();
  uint8_t currentCase = checkIrCase();
  switch (currentCase)
  {
  // ☖ ☖ ☖ ☖ ☖
  case 1:
    if (developerMode) Serial.print("Case 1: ☖ ☖ ☖ ☖ ☖");
    forwardAll();
    break;
  // ☖ ☖ ☗ ☖ ☖
  case 2:
    if (developerMode) Serial.print("Case 2: ☖ ☖ ☗ ☖ ☖");
    forwardAll();
    break;
  // ☖ ☖ ☗ ☗ ☖
  case 3:
    if (developerMode) Serial.print("Case 3: ☖ ☖ ☗ ☗ ☖");
    rightAdjust();
    break;
  // ☖ ☗ ☗ ☖ ☖
  case 4:
    if (developerMode) Serial.print("Case 4: ☖ ☗ ☗ ☖ ☖");
    leftAdjust();
    break;
  // ☖ ☖ ☖ ☗ ☗
  case 5:
    if (developerMode) Serial.print("Case 5: ☖ ☖ ☖ ☗ ☗");
    rightFastAdjust();
    break;
  // ☗ ☗ ☖ ☖ ☖
  case 6:
    if (developerMode) Serial.print("Case 6: ☗ ☗ ☖ ☖ ☖");
    leftFastAdjust();
    break;
  // ☖ ☖ ☖ ☖ ☗
  case 7:
    if (developerMode) Serial.print("Case 7: ☖ ☖ ☖ ☖ ☗");
    rightStraight();
    delay(straightMoveDelay);
    forwardAll();
    delay(straightStopDelay);
    break;
  // ☗ ☖ ☖ ☖ ☖
  case 8:
    if (developerMode) Serial.print("Case 8: ☗ ☖ ☖ ☖ ☖");
    leftStraight();
    delay(straightMoveDelay);
    forwardAll();
    delay(straightStopDelay);
    break;
  // ☖ ☖ ☗ ☗ ☗
  case 9:
    if (developerMode) Serial.print("Case 9: ☖ ☖ ☗ ☗ ☗");
    rightStraight();
    delay(straightMoveDelay);
    forwardAll();
    delay(straightStopDelay);
    break;
  // ☗ ☗ ☗ ☖ ☖
  case 10:
    if (developerMode) Serial.print("Case 10: ☗ ☗ ☗ ☖ ☖");
    leftStraight();
    delay(straightMoveDelay);
    forwardAll();
    delay(straightStopDelay);
    break;
  // ☗ ☗ ☗ ☗ ☗
  case 11:
    if (developerMode) Serial.print("Case 11: ☗ ☗ ☗ ☗ ☗");
    stopAll();
    break;
  }
  if (developerMode) Serial.println("");
  delay(35);
  stopAll();
  delay(100);
}

uint16_t irSensorRead(int irNumber)
{
  uint16_t irReading = analogRead(irSensorPins[irNumber - 1]) * irMultiplicationFactors[irNumber - 1];
  if (useLeds && irReading < irBorder) digitalWrite(ledPins[irNumber - 1], HIGH);
  else digitalWrite(ledPins[irNumber - 1], LOW);
  return irReading;
}

bool* getIrReadings() {
  static bool irReadings[5];
  for (int i = 0; i < 5; i++) {
    irReadings[i] = irSensorRead(i + 1) < irBorder ? true : false;
  }
  return irReadings; 
}

bool getIrReading(int irNumber)
{
  return irSensorRead(irNumber) < irBorder ? false : true;
}

float ultraSensorRead(int ultraNumber)
{
  digitalWrite(trigPins[ultraNumber - 1], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[ultraNumber - 1], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[ultraNumber - 1], LOW);
  return pulseIn(echoPins[ultraNumber - 1], HIGH) * 0.0343 / 2;
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

void motorControl(char side, char direction, uint8_t speedL, uint8_t speedR)
{
  if (side == 'L')
  {
    switch (direction)
    {
    case 'F':
      analogWrite(motorPins[0], speedL);
      digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], LOW);
      break;
    case 'B':
      analogWrite(motorPins[0], speedL);
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
      analogWrite(motorPins[3], speedR);
      digitalWrite(motorPins[4], HIGH);
      digitalWrite(motorPins[5], LOW);
      break;
    case 'B':
      analogWrite(motorPins[3], speedR);
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
      analogWrite(motorPins[0], speedL);
      analogWrite(motorPins[3], speedR);
      digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], LOW);
      digitalWrite(motorPins[4], HIGH);
      digitalWrite(motorPins[5], LOW);
      break;
    case 'B':
      analogWrite(motorPins[0], speedL);
      analogWrite(motorPins[3], speedR);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], HIGH);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], HIGH);
      break;
    case 'L':
      analogWrite(motorPins[0], speedL);
      analogWrite(motorPins[3], speedR);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], HIGH);
      digitalWrite(motorPins[4], HIGH);
      digitalWrite(motorPins[5], LOW);
      break;
    case 'R':
      analogWrite(motorPins[0], speedL);
      analogWrite(motorPins[3], speedR);
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
    pinMode(ledPins[i], OUTPUT);
  }
  for (int i = 0; i < 6; i++)
  {
    pinMode(motorPins[i], OUTPUT);
  }
  for (int i = 0; i < 3; i++)
  {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
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
  mpu.upsideDownMounting = isGyroscopeMountedUpsideDown;
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
  if (developerMode)
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
  
    sensorData["leftUltra"] = (float) ultraSensorRead(1);
    // sensorData["centerUltra"] = (float) ultraSensorRead(2);
    sensorData["rightUltra"] = (float) ultraSensorRead(3);
  
    sensorData["ir1"] = (int) irSensorRead(1);
    sensorData["ir2"] = (int) irSensorRead(2);
    sensorData["ir3"] = (int) irSensorRead(3);
    sensorData["ir4"] = (int) irSensorRead(4);
    sensorData["ir5"] = (int) irSensorRead(5);
  
    String sensorDataString = JSON.stringify(sensorData);
  
    Serial.println(sensorDataString);
  }
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
        motorControl('B', 'F', motorLVelocity, motorRVelocity);
        break;
      case 'B':
        motorControl('B', 'B', motorLVelocity, motorRVelocity);
        break;
      case 'L':
        motorControl('B', 'L', motorLVelocity, motorRVelocity);
        break;
      case 'R':
        motorControl('B', 'R', motorLVelocity, motorRVelocity);
        break;
      case 'S':
        motorControl('B', 'S', motorLVelocity, motorRVelocity);
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

bool compareIrCase(bool irReadings[], bool irCase[]) {
  for (int i = 0; i < 5; i++) {
    if (irReadings[i] != irCase[i]) {
      return false;
    }
  }
  return true;
}

uint8_t checkIrCase() {
  bool * irReadings = getIrReadings();

  bool gapCase[5] =           {0, 0, 0, 0, 0};
  bool straightCase[5] =      {0, 0, 1, 0, 0};
  bool smoothLeftCase[5] =    {0, 0, 1, 1, 0};
  bool smoothRightCase[5] =   {0, 1, 1, 0, 0};
  bool leftCase[5] =          {0, 0, 0, 1, 1};
  bool rightCase[5] =         {1, 1, 0, 0, 0};
  bool farLeftCase[5] =       {0, 0, 0, 0, 1};
  bool farRightCase[5] =      {1, 0, 0, 0, 0};
  bool straightLeftCase[5] =  {0, 0, 1, 1, 1};
  bool straightRightCase[5] = {1, 1, 1, 0, 0};
  bool intersectionCase[5] =  {1, 1, 1, 1, 1};

  if (compareIrCase(irReadings, gapCase))                 return 1;
  else if (compareIrCase(irReadings, straightCase))       return 2;
  else if (compareIrCase(irReadings, smoothRightCase))    return 3;
  else if (compareIrCase(irReadings, smoothLeftCase))     return 4;
  else if (compareIrCase(irReadings, rightCase))          return 5;
  else if (compareIrCase(irReadings, leftCase))           return 6;
  else if (compareIrCase(irReadings, farRightCase))       return 7;
  else if (compareIrCase(irReadings, farLeftCase))        return 8;
  else if (compareIrCase(irReadings, straightRightCase))  return 9;
  else if (compareIrCase(irReadings, straightLeftCase))   return 10;
  else if (compareIrCase(irReadings, intersectionCase))   return 11;
}

void forwardAll()      { motorControl('B', 'F', motorLVelocity, motorRVelocity); }
void forwardLeft()     { motorControl('L', 'F', motorLVelocity, motorRVelocity); }
void forwardRight()    { motorControl('R', 'F', motorLVelocity, motorRVelocity); }
void backwardAll()     { motorControl('B', 'B', motorLVelocity, motorRVelocity); }
void backwardLeft()    { motorControl('L', 'B', motorLVelocity, motorRVelocity); }
void backwardRight()   { motorControl('R', 'B', motorLVelocity, motorRVelocity); }
void stopAll()         { motorControl('B', 'S', motorLVelocity, motorRVelocity); }
void stopLeft()        { motorControl('L', 'S', motorLVelocity, motorRVelocity); }
void stopRight()       { motorControl('R', 'S', motorLVelocity, motorRVelocity); }

void leftStraight()    { motorControl('B', 'R', motorLVelocity, (fastAdjustmentRVelocity + motorRVelocity) / 2); }
void rightStraight()   { motorControl('B', 'L', (fastAdjustmentLVelocity + motorLVelocity) / 2, motorRVelocity); }

void leftAdjust()      { motorControl('B', 'F', motorLVelocity, adjustmentRVelocity); }
void rightAdjust()     { motorControl('B', 'F', adjustmentLVelocity, motorRVelocity); }

void leftFastAdjust()  { motorControl('B', 'F', fastAdjustmentLVelocity, adjustmentRVelocity); }
void rightFastAdjust() { motorControl('B', 'F', adjustmentLVelocity, fastAdjustmentRVelocity); }