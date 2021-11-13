#include "AsyncAnalog.h"
#include "TimerOne.h"

char _printfBuffer[64];

#define printf(fmt, ...) \
  snprintf(_printfBuffer, 64, fmt, ##__VA_ARGS__); \
  Serial.print(_printfBuffer);

// Constants
constexpr int inputPin = 2; // A2 connected to phototransistor
constexpr int bufferSize = 750;
constexpr int samplingRate = 5000; // 5khz
constexpr float targetFreq = 200; // 200hz
constexpr float threshold = 30;

unsigned int writePtr = 0;
uint16_t adcBuffer[bufferSize];

ISR(ADC_vect) {
  adcBuffer[writePtr] = analogReadGetValue();
  writePtr = (writePtr + 1) % bufferSize;
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

  void init() {
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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  analogReadSetup(inputPin);
  Timer1.initialize(1000000 / samplingRate);
  Timer1.attachInterrupt(analogReadStart);
  Serial.begin(115200);

  goertzel.init();
  delay(1000);
}

void loop() {
  {
    int_guard guard {};
    goertzel.from(adcBuffer);
  }

  float magnitude = goertzel.processAll();
  char bar[40] = {};
  int numBars = (int) magnitude / 2;
  memset(bar, '=', numBars > 39 ? 39 : numBars);
  // printf("Received magnitude: %d/100\n", (int)(magnitude * 100));

  bool toneDetected = magnitude > threshold;

  printf("M=%03d |%c|%s>\n", (int)(magnitude), toneDetected ? '*' : ' ', bar);
  digitalWrite(LED_BUILTIN, toneDetected);

  // Delay for a little bit
  delay(300);
}
