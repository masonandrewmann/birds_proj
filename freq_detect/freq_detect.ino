#include "TimerOne.h"
#include "AsyncAnalog.h"
#include <SPI.h>
#include <DAC_MCP49xx.h>
#define SS_PIN 10 //define chip select pin
#define TABLESIZE 50



#define printf(fmt, ...) \
  snprintf(_printfBuffer, 64, fmt, ##__VA_ARGS__);\
  Serial.print(_printfBuffer);

int SineValues[TABLESIZE]; 
float pointerInc;
float pointerVal = 0;
int outVal;
float outFreq = 400;

//Set up the DAC
DAC_MCP49xx dac(DAC_MCP49xx::MCP4901, SS_PIN);

char _printfBuffer[64]; 

// Constants
constexpr int inputPin = A0; // A0 connected to microphone
constexpr int bufferSize = 256;
constexpr int samplingRate = 2500; // 5khz
//constexpr float targetFreq = 200; // 200hz
constexpr float threshold = 30;

//target frequencies:
//constexpr float targetC4 = 261.63; // C4
//constexpr float targetD4 = 293.66; // D4
//constexpr float targetE4 = 329.63; // E4
//constexpr float targetG4 = 392.00; // G4
//constexpr float targetA4 = 440.00; // A4
//constexpr float targetC5 = 523.25; // C5
float mags[6];

unsigned int writePtr = 0;
uint16_t adcBuffer[bufferSize];

ISR(ADC_vect) {
  adcBuffer[writePtr] = analogReadGetValue();
  writePtr = (writePtr + 1) % bufferSize;
  cycle();
  dac.output(outVal);
}

// Just so part of our program doesn't get interruptted
struct int_guard {
  int_guard() { noInterrupts(); }
  ~int_guard() { interrupts(); }
};

// Group everything here
template<typename FLOATING = float, typename SAMPLE = uint16_t>
struct GoertzelBuffer {
  SAMPLE currentSamples[bufferSize];
  FLOATING coeff, Q1, Q2, sine, cosine;

  void from(SAMPLE buffer[bufferSize], int startIndex = 0) {
    memcpy(currentSamples, buffer + startIndex, sizeof(SAMPLE) * (bufferSize - startIndex));
    memcpy(currentSamples + startIndex, buffer, sizeof(SAMPLE) * startIndex);
  }

  void init(FLOATING targetFreq) {
    int k;
    FLOATING floatN;
    FLOATING omega;

    floatN = (FLOATING) bufferSize;
    k = (int) (0.5 + ((floatN * targetFreq) / samplingRate));
    omega = (2.0 * PI * k) / floatN;
    sine = sin(omega);
    cosine = cos(omega);
    coeff = 2.0 * cosine;

    printf("For samplingRate = %d", (int) samplingRate);
    printf(" N = %d", (int) bufferSize);
    printf(" and FREQUENCY = %d,\n", (int) targetFreq);
    printf("k = %d and coeff = %d/100\n\n", (int) k, (int) (coeff * 100));
  }


  void ProcessSample(SAMPLE sample) {
    FLOATING Q0;
    Q0 = coeff * Q1 - Q2 + (FLOATING) sample;
    Q2 = Q1;
    Q1 = Q0;
  }

  void updateFreq(FLOATING newFreq){
    int k;
    FLOATING floatN;
    FLOATING omega;

    floatN = (FLOATING) bufferSize;
    k = (int) (0.5 + ((floatN * newFreq) / samplingRate));
    omega = (2.0 * PI * k) / floatN;
    sine = sin(omega);
    cosine = cos(omega);
    coeff = 2.0 * cosine;
  }

  /* Basic Goertzel */
  /* Call this routine after every block to get the complex result. */
  void GetRealImag(FLOATING *realPart, FLOATING *imagPart) {
    *realPart = (Q1 - Q2 * cosine);
    *imagPart = (Q2 * sine);
  }

  /* Optimized Goertzel */
  /* Call this after every block to get the RELATIVE magnitude squared. */
  FLOATING GetMagnitudeSquared(void) {
    FLOATING result;

    result = Q1 * Q1 + Q2 * Q2 - Q1 * Q2 * coeff;
    return result;
  }

  void ResetGoertzel(void) {
    Q2 = 0;
    Q1 = 0;
  }

  float processAll() {
    ResetGoertzel();
    for (const auto& sample : currentSamples) {
      ProcessSample(sample);
    }
    auto mag2 = GetMagnitudeSquared();
    return sqrt(mag2);
  }
};

GoertzelBuffer<> goertzel {};

void cycle(){
  pointerInc = TABLESIZE * (outFreq / samplingRate);
  outVal = SineValues[(int)pointerVal];
  pointerVal += pointerInc;
  if(pointerVal > TABLESIZE) pointerVal -= TABLESIZE;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  analogReadSetup(inputPin);
  Timer1.initialize(1000000 / samplingRate);
  Timer1.attachInterrupt(analogReadStart);
  Serial.begin(115200);

    // calculate sine wavetable
  float RadAngle;                           // Angle in Radians
  for(int MyAngle=0;MyAngle<TABLESIZE;MyAngle++) {
    RadAngle=MyAngle*(2*PI)/TABLESIZE;               // angle converted to radians
    SineValues[MyAngle]=(sin(RadAngle)*127)+128;  // get the sine of this angle and 'shift' to center around the middle of output voltage range
  }

  //
  pointerInc = TABLESIZE * (outFreq / samplingRate);
  pointerVal = map(0, 0, TWO_PI, 0, TABLESIZE - 1);

  goertzel.init(261.63);
  delay(1000);
}

void loop() {
  {
    int_guard guard {};
    goertzel.from(adcBuffer);
  }

  //process for all frequencies
  // C4
  goertzel.updateFreq(261.63);
  mags[0] = (goertzel.processAll() * 10);
  // D4
  goertzel.updateFreq(293.66);
  mags[1] = (goertzel.processAll() * 10);
  // E4
  goertzel.updateFreq(329.63);
  mags[2] = (goertzel.processAll() * 10);
  // G4
  goertzel.updateFreq(392.00);
  mags[3] = (goertzel.processAll() * 10);
  // A4
  goertzel.updateFreq(440.00);
  mags[4] = (goertzel.processAll() * 10);
  // C5
  goertzel.updateFreq(523.25);
  mags[5] = (goertzel.processAll() * 10);

//  printf(" /n");
  Serial.print("C4: ");
  Serial.print(mags[0]);
  Serial.print(" D4: ");
  Serial.print(mags[1]);
  Serial.print(" E4: ");
  Serial.print(mags[2]);
  Serial.print(" G4: ");
  Serial.print(mags[3]);
  Serial.print(" A4: ");
  Serial.print(mags[4]);
  Serial.print(" C5: ");
  Serial.println(mags[5]);
//  printf("C4: %d, D4: %d, E4: %d, G4: %d, A4: %d, C5: %d\n", (int)(mags[0] * 100), (int)(mags[1] * 100), (int)(mags[2] * 100), (int)(mags[3] * 100), (int)(mags[4] * 100), (int)(mags[5] * 100)); 
  
//  char bar[40] = {};
//  int numBars = (int) magnitude / 2;
//  memset(bar, '=', numBars > 39 ? 39 : numBars);
//   printf("Received magnitude: %d/100\n", (int)(magnitude * 100));

//  bool toneDetected = magnitude > threshold;

//  printf("M=%03d |%c|%s>\n", (int)(magnitude), toneDetected ? '*' : ' ', bar);
//  digitalWrite(LED_BUILTIN, toneDetected);

  // Delay for a little bit
//  delay(300);
}
