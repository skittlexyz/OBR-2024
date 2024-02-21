#include "SKTLF.h"
#include "Arduino.h"
#include "SKTLDR.h"
SKTLF::SKTLF(SKTLDR& ldr1, SKTLDR& ldr2, SKTLDR& ldr3) : _ldr1(ldr1), _ldr2(ldr2), _ldr3(ldr3) {
  _ldr1 = ldr1;
  _ldr2 = ldr2;
  _ldr3 = ldr3;
}
char SKTLF::getState() {
  // Left Forward Right Backward Null
  if (!_ldr1.getRead() && !_ldr2.getRead() && !_ldr3.getRead()) { // 000
    return('F');
  } else if (!_ldr1.getRead() && !_ldr2.getRead() && _ldr3.getRead()) { // 001
    return('R');
  } else if (_ldr1.getRead() && !_ldr2.getRead() && !_ldr3.getRead()) { // 100
    return('L');
  } else if (!_ldr1.getRead() && _ldr2.getRead() && !_ldr3.getRead()) { // 010
    return('F');
  } else if (!_ldr1.getRead() && _ldr2.getRead() && _ldr3.getRead()) { // 011
    return('F');
  } else if (_ldr1.getRead() && _ldr2.getRead() && !_ldr3.getRead()) { // 110
    return('F');
  } else if (_ldr1.getRead() && _ldr2.getRead() && _ldr3.getRead()) { // 111
    return('F');
  }
  return(NULL);
}
int SKTLF::getSingleLimit(int ldrNumber) {
  switch (ldrNumber)
  {
  case 1:
    return _ldr1.getLimit();
    break;
  case 2:
    return _ldr2.getLimit();
    break;
  case 3:
    return _ldr3.getLimit();
    break;
  }
  return NULL;
}
void SKTLF::setSingleLimit(int ldrNumber, int limit) {
  switch (ldrNumber)
  {
  case 1:
    _ldr1.setLimit(limit);
    break;
  case 2:
    _ldr2.setLimit(limit);
    break;
  case 3:
    _ldr3.setLimit(limit);
    break;
  }
}
void SKTLF::setAllLimits(int limit) {
  _ldr1.setLimit(limit);
  _ldr2.setLimit(limit);
  _ldr3.setLimit(limit);
}
int SKTLF::getSingleRaw(int ldrNumber) {
  switch (ldrNumber)
  {
  case 1:
    return _ldr1.getRaw();
    break;
  case 2:
    return _ldr2.getRaw();
    break;
  case 3:
    return _ldr3.getRaw();
    break;
  }
  return NULL;
}
bool SKTLF::getSingleRead(int ldrNumber) {
  switch (ldrNumber)
  {
  case 1:
    return _ldr1.getRead();
    break;
  case 2:
    return _ldr2.getRead();
    break;
  case 3:
    return _ldr3.getRead();
    break;
  }
  return NULL; 
}