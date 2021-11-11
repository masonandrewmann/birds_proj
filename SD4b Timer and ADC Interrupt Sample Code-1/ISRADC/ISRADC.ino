#include "AsyncADC.h"

// Read AsyncADC.h for more information about how to use
// each function.

#define LEDPIN 13

volatile int analogValue;
volatile int dataReady = 0;
unsigned long lastadc = 0;

// This subroutine will be called when the ADC is
// finished (after you called analogReadStart()).
ISR(ADC_vect) {
  analogValue = analogReadGetValue();
  dataReady = 1;
}

void setup() {
  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  pinMode(LEDPIN, OUTPUT);    

  Serial.begin(9600);

  analogReadSetup(0);  
  
  lastadc = millis();
}

#define ADC_PERIOD 100 // ms

void loop() {
  // Main code loop
  if(millis() >= lastadc+ADC_PERIOD) {
    lastadc = lastadc + ADC_PERIOD;
    analogReadStart();
    digitalWrite( LEDPIN, digitalRead( LEDPIN ) ^ 1 );
  }
  
  if(dataReady) {
    Serial.println(analogValue); 
    dataReady = 0;
  }
}
 
