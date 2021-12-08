// Analog IO

#ifndef ASYNCANALOG_h
#define ASYNCANALOG_h

#include "Arduino.h"

void analogReadSetup(uint8_t pin);
void analogReadStart();
int analogReadStatus();
int analogReadGetValue();

#endif
