#include "Arduino.h"
#include "SKTSTEPPER.h"
SKTSTEPPER::SKTSTEPPER(char axis) {
  digitalWrite(8, LOW);
  switch (axis) {
  case 'X':
    _stepPin = 2;
    _dirPin = 5;
    break;
  case 'Y':
    _stepPin = 3;
    _dirPin = 6;
    break;
  case 'Z':
    _stepPin = 4;
    _dirPin = 7;
    break;
  }
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
}
void SKTSTEPPER::step(int times, bool dir, int interval) {
  for (int i = 0; i < times; i++) {
    digitalWrite(_dirPin, dir);
    digitalWrite(_stepPin, HIGH);
    delay(interval);
    digitalWrite(_stepPin, LOW);
    delay(interval);
  }
}