#include "SKTLF.h"
#include "SKTSTEPPER.h"
#include "SKTLDR.h"

SKTSTEPPER motorA('X');
SKTSTEPPER motorB('Y');

SKTLDR ldr1(A8, A11);
SKTLDR ldr2(A9, A11);
SKTLDR ldr3(A10, A11);

SKTLF lf(ldr1, ldr2, ldr3);

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print(analogRead(A11));
  Serial.print(" | ");
  Serial.print(lf.getState());
  Serial.print(" | ");
  Serial.print(lf.getSingleRaw(1));
  Serial.print(" | ");
  Serial.print(lf.getSingleRaw(2));
  Serial.print(" | ");
  Serial.println(lf.getSingleRaw(3));
  switch (lf.getState()) {
    case 'F':
      motorA.step(5, true, 50);
      motorB.step(5, true, 50);
      break;
    case 'L':
      motorA.step(5, true, 50);
      motorB.step(5, false, 50);
      break;
    case 'R':
      motorA.step(5, false, 50);
      motorB.step(5, true, 50);
      break;
  }
  delay(1);
}
