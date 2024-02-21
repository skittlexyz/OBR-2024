#include "Arduino.h"
#include "SKTLDR.h"
SKTLDR::SKTLDR(int inputPin, int sensPin) {
  pinMode(inputPin, INPUT);
  pinMode(sensPin, INPUT);
  _inputPin = inputPin;
  _sensPin = sensPin;
  if (_sensPin == NULL) {
    _usePot = false;
  } else {
    _usePot = true;
  }
}
int SKTLDR::getLimit() {
  if (_usePot) {
    return analogRead(_sensPin);
  }
  return _limit;
}
void SKTLDR::setLimit(int limit) {
  _limit = limit;
}
int SKTLDR::getRaw() {
  return analogRead(_inputPin);
}
bool SKTLDR::getRead() {
  _limit = getLimit();
  if (analogRead(_inputPin) > _limit) {
    return false;
  } else if (analogRead(_inputPin) <= _limit) {
    return true;
  }
  return false;
}