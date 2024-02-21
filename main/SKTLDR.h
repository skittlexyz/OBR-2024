#ifndef SKTLDR_h
#define SKTLDR_h
#include "Arduino.h"
class SKTLDR {
private:
  int _inputPin;
  int _sensPin;
  int _limit = 250;
  bool _usePot;
public:
  SKTLDR(int inputPin, int sensPin);
  int getLimit();
  void setLimit(int limit);
  int getRaw();
  bool getRead();
};
#endif