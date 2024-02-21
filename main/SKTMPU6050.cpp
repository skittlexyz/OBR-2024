#include "Arduino.h"
#include <Wire.h>
#include "SKTMPU6050.h"
SKTMPU6050::SKTMPU6050(int address) {
  _address = address;
}
void SKTMPU6050::begin() {
  Wire.begin();
  Wire.beginTransmission(_address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}
void SKTMPU6050::refreshValues() {
  Wire.beginTransmission(_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(_address, 14, true);
  _AcX = Wire.read() << 8 | Wire.read();
  _AcY = Wire.read() << 8 | Wire.read();
  _AcZ = Wire.read() << 8 | Wire.read();
  int _xAng = map(_AcX, _minVal, _maxVal, -90, 90);
  int _yAng = map(_AcY, _minVal, _maxVal, -90, 90);
  int _zAng = map(_AcZ, _minVal, _maxVal, -90, 90);
  _x = RAD_TO_DEG * (atan2(-_yAng, -_zAng) + PI);
  _y = RAD_TO_DEG * (atan2(-_xAng, -_zAng) + PI);
  _z = RAD_TO_DEG * (atan2(-_yAng, -_xAng) + PI);
}
int SKTMPU6050::getX() {
  refreshValues();
  return round(_x);
}
int SKTMPU6050::getY() {
  refreshValues();
  return round(_y);
}
int SKTMPU6050::getZ() {
  refreshValues();
  return round(_z);
}