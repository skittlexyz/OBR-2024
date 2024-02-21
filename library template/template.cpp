#include "Arduino.h”
#include "template.h"
template::template(int pin) {
	pinMode(pin, OUTPUT);
	_pin = pin;
}
void template::blink(int blinkRate){
  digitalWrite(_pin, HIGH);
  delay(blinkRate);
  digitalWrite(_pin, LOW);
  delay(blinkRate);
}