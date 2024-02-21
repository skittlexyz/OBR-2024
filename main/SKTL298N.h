#ifndef SKTL298N_h
#define SKTL298N_h
#include "Arduino.h"
class SKTL298N {
public:
  SKTL298N(int enPin, int aPin, int bPin);
  void setSpeed(int pwm);
  void moveForward();
  void moveBackward();
  void stop();
private:
  int _enPin;
  int _aPin;
  int _bPin;
  int _pwm;
};
#endif