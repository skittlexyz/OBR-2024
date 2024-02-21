#ifndef SKTLF_h
#define SKTLF_h
#include "Arduino.h"
#include "SKTLDR.h"
class SKTLF {
  public:
    SKTLF(SKTLDR& ldr1, SKTLDR& ldr2, SKTLDR& ldr3);
    char getState();
    int getSingleLimit(int ldrNumber);
    void setSingleLimit(int ldrNumber, int limit);
    void setAllLimits(int limit);
    int getSingleRaw(int ldrNumber);
    bool getSingleRead(int ldrNumber);
  private:
    SKTLDR& _ldr1;
    SKTLDR& _ldr2;
    SKTLDR& _ldr3;
    char _state;
};
#endif