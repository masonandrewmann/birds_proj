#include <SPI.h>
#include <DAC_MCP49xx.h>

#define SS_PIN 10 //define chip select pin

//Set up the DAC
DAC_MCP49xx dac(DAC_MCP49xx::MCP4901, SS_PIN);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 255; i = i + 11){
    dac.output(i);
//    delayMicroseconds(1); 
  }
//  dac.output(255);
//  delayMicroseconds(500);
//  dac.output(0);
//  delayMicroseconds(900);
}
