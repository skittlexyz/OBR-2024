#ifndef SKTSTEPPER_h
#define SKTSTEPPER_h
#include "Arduino.h"
class SKTSTEPPER {
public:
  SKTSTEPPER(char axis);
  void step(int times, bool dir, int delay);
private:
  int _stepPin;
  int _dirPin;
};
#endif