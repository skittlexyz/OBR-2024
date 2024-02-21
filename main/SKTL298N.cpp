#include "Arduino.h"
#include "SKTL298N.h"
SKTL298N::SKTL298N(int enPin, int aPin, int bPin) {
	pinMode(enPin, OUTPUT);
	pinMode(aPin, OUTPUT);
	pinMode(bPin, OUTPUT);
  _enPin = enPin;
  _aPin = aPin;
  _bPin = bPin;
  digitalWrite(_aPin, LOW);
  digitalWrite(_bPin, LOW);
}
void SKTL298N::setSpeed(int pwm){
  _pwm = pwm;
}
void SKTL298N::moveForward(){
  analogWrite(_enPin, _pwm);
  digitalWrite(_aPin, HIGH);
  digitalWrite(_bPin, LOW);
}
void SKTL298N::moveBackward(){
  analogWrite(_enPin, _pwm);
  digitalWrite(_aPin, LOW);
  digitalWrite(_bPin, HIGH);
}
void SKTL298N::stop(){
  digitalWrite(_enPin, LOW);
  digitalWrite(_aPin, LOW);
  digitalWrite(_bPin, LOW);
}