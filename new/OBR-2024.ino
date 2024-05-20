/* OBR 2024 Code - Nióbio */
/* Made by Moisés Corrêa Gomes */

/* Libraries section */
#include <U8g2lib.h>
#include "Adafruit_TCS34725softi2c.h"
#include <MPU6050_light.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/* General configurations section */
int motorVelocity = 150;
#define colorVelocity TCS34725_INTEGRATIONTIME_50MS

/* Pin declaration and variables section */
// IR Sensor pins
uint8_t irSensorPins[8] = {22,23,24,25,26,27,28,29};
// Ultrassonic Pins {trig, echo, ...}
uint8_t ultraPins[8] = {35,34,37,36,39,38}; 
// Grouped Stepper Motor pins {pwm min, a pin, b pin, ...}
uint8_t motorPins[6] = {_,_,_,_,_,_};
// TCS34725 Pins {sda, scl, ...}
uint8_t colorPins[4] = {31,30,33,32};

/* Objects declaration*/
// TCS34725
Adafruit_TCS34725softi2c leftColor = Adafruit_TCS34725softi2c(colorVelocity, TCS34725_GAIN_1X, colorPins[0], colorPins[1]);
Adafruit_TCS34725softi2c rightColor = Adafruit_TCS34725softi2c(colorVelocity, TCS34725_GAIN_1X, colorPins[2], colorPins[3]);
// MPU6050
MPU6050 mpu;

/* Function declaration section */
bool irSensorRead(int pin);
float ultraSensorRead(int trigPin, int echoPin);
int colorSensorRead(char side, char color);
float mpuSensorRead(char axis);
void motorControl(char side, char direction);
bool pinsSetup();
bool colorSetup();
bool mpuSetup();

/* Main code section */
void setup() {
    
}

void loop() {
    
}

bool irSensorRead(int pin) {
    return digitalRead(pin);
}

float ultraSensorRead(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH) * 0.0343 / 2;
}

int colorSensorRead(int side, int color) {
  uint16_t r, g, b, c;
  switch (side) {
  case 'L':
    leftColor.getRawData(&r, &g, &b, &c);
    switch (color) {
    case 'R':
      return r;
      break;
    case 'G':
      return g;
      break;
    case 'B':
      return b;
      break;
    case 'C':
      return c;
      break;
    }
    break;  
  case 'R':
    leftColor.getRawData(&r, &g, &b, &c);
    switch (color) {
    case 'R':
      return r;
      break;
    case 'G':
      return g;
      break;
    case 'B':
      return b;
      break;
    case 'C':
      return c;
      break;
    }
    break;
  }
}

