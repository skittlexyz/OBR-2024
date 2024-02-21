#ifndef SKTMPU6050_h
#define SKTMPU6050_h
#include "Arduino.h"
class SKTMPU6050 {
private:
  int _address;
  int16_t _AcX, _AcY, _AcZ, _Tmp, _GyX, _GyY, _GyZ;
  int _minVal = 265;
  int _maxVal = 402;
  double _x, _y, _z;
  void refreshValues();
public:
  SKTMPU6050(int address);
  void begin();
  int getX();
  int getY();
  int getZ();
};
#endif