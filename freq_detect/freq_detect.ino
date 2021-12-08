#include "TimerOne.h"
#include "AsyncAnalog.h"
#include <SPI.h>
#include <DAC_MCP49xx.h>
#define SS_PIN 10 //define chip select pin
#define TABLESIZE 50
#include <MarkovChain.h>

#define LED_1 PD5
#define LED_2 PD6
#define BUTTON_PIN PD7
#define LED_4 PB0

#define printf(fmt, ...) \
  snprintf(_printfBuffer, 64, fmt, ##__VA_ARGS__);\
  Serial.print(_printfBuffer);
  
MarkovChain chain;

int SineValues[TABLESIZE]; 
float pointerInc;
float pointerVal = 0;
int outVal;

int outInd = 0;
float freqs [] = {261.63, 293.66, 329.63, 392.00, 440.00, 523.25};
//char elements [] = {'a','b','c','d','e','f'};


//hiiii

//Set up the DAC
DAC_MCP49xx dac(DAC_MCP49xx::MCP4901, SS_PIN);

//Set up ASR envelope
//int attack = 500;
//int sus = 1000;
//int decay = 1000;
int envTimes[4] = {500, 500, 500, 0}; // [attack, sustain, release, PLACEHOLDER]
float envVal = 1;
boolean noteActive = false;
unsigned long noteEnd = 0;
byte envState = 0; // 0 for attack, 1 for sustain, 2 for release, 3 for inactive
unsigned long sectionStart = 0;
unsigned long sectionEnd = 0;

//note clock
unsigned long nextNote = 0;

//note control
boolean listening = true;
unsigned long listenTimer = 2000;
int listenLength = 2000;
byte listenCount = 0;
float outFreq = 100;


char _printfBuffer[64]; 

// Constants
constexpr int inputPin = A0; // A0 connected to microphone
constexpr int bufferSize = 256;
constexpr int samplingRate = 2500; // 1250 Nyquist
//sampling rate is mainly so low to have precise frequency resolution on the Goertzel algorithm. bin size = Fs/N = 2500 / 256 = 9.76Hz
//time resolution is 256 * 2500 = 0.1024ms

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
float magsMax[6] = {0, 0, 0, 0, 0, 0};

float pitches[6];

unsigned int writePtr = 0;
uint16_t adcBuffer[bufferSize];

ISR(ADC_vect) {
  //read from microphone
  adcBuffer[writePtr] = analogReadGetValue();
  writePtr = (writePtr + 1) % bufferSize;
  //output to speaker
  if (noteActive){
    cycle();
    dac.output(outVal);
  } else {
    dac.output(0);
  } 
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
  void GetRealImag(FLOATING *realPart, FLOATING * imagPart) {
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
  pointerInc = TABLESIZE * (freqs[outInd] / samplingRate);
  //grab sample from wavetable
  outVal = SineValues[(int)pointerVal];

  //check envelope state
  if (millis() > sectionEnd) {
    envState++;
    sectionStart = millis();
    sectionEnd = sectionStart + envTimes[envState];
  }
  //determine envelope value
  switch (envState){
    case 0: //attack
      envVal = (float)(millis() - sectionStart)  / (float)(sectionEnd - sectionStart);
    break;
    case 1: //sustain
      envVal = 1;
    break;

    case 2: //release
      envVal = 1 + (float)(millis() - sectionStart) * (-1) / (float)(sectionEnd - sectionStart);
    break;
    case 3: //end the note
      noteActive = 0;
      listening = true;
      digitalWrite(LED_1, LOW);
    break;
  }
  //multiply by envelope value
  outVal *= envVal;
  //increment wavetable pointer
  pointerVal += pointerInc;
  if(pointerVal > TABLESIZE) pointerVal -= TABLESIZE;
}


char states[] = {'a','b','c','d','e'};
//float tmatrix[][] = {{1,0,0,0,0},{0,1,0,0,0},{0,0,1,0,0},{0,0,0,1,0},{0,0,0,0,1}};
int lenStates = 5; //CHANGE THIS MANUALLY IF WE INCREASE THE NUMBER OF NOTES

void markov(char curVal) {
//  find the current value's index in the states[] array
  int wantedpos = -1;
  for (int i=0; i<lenStates; i++) {
    if (curVal == states[i]) {
      wantedpos = i;
      break;
    }
  }
  
  float curDist[] = {0,0,0,0,0};
  // put a 1 in the current position so we can multiply this matrix by the probability matrix
  curDist[wantedpos] = 1;
  // this number is wrong i think maybe but i wanna do it on paper to figure it out.
  
  
}

//void trainMarkov(){
//  //Training set to create the transition matrix. It has 3 different sequences.  
//  char ** trainingSet;
//  trainingSet = (char **) malloc(3*sizeof(char *));
//  for(int i = 0; i < 3; i++)
//    trainingSet[i] = (char *) malloc(5*sizeof(char));
// 
//  //Sequence 0
//  trainingSet[0][0] = 'a';  
//  trainingSet[0][1] = 'b';  
//  dtrainingSet[0][2] = 'c';  
//  trainingSet[0][3] = 'c';  
//  trainingSet[0][4] = '\0'; 
//  //Sequence 1
//  trainingSet[1][0] = 'a';  
//  trainingSet[1][1] = 'a';  
//  trainingSet[1][2] = 'a';  
//  trainingSet[1][3] = 'a';  
//  trainingSet[1][4] = '\0'; 
//  //Sequence 2
//  trainingSet[2][0] = 'b';  
//  trainingSet[2][1] = 'a';  
//  trainingSet[2][2] = 'b';  
//  trainingSet[2][3] = 'c';  
//  trainingSet[2][4] = '\0';  
//
//  // The sequences are composed by three elements: a, b and c
//  char elements[] = {'a', 'b', 'c'};
//  
//  //We calculate the probability of a element apearing after 'a'  
//  double* probs = chain.getNextTransitions('a', elements, 3, trainingSet, 3);
//  
//  Serial.println("Probability of appearing after element /'a/'");
//  for (int i = 0; i < 3; i++){
//    Serial.print(elements[i]);
//    Serial.print(probs[i]);
//    Serial.println();
//  }
//  
//  
//}

//void nextPitch(){
//remember we have freqs[] and elements[]
//  float curPitch = freqs[outInd];
//  
//  char curEl = elements[outInd];
//  Serial.print(curEl);
//  double* probs = chain.getNextTransitions(curEl, elements, 6, trainingSet, 20);
//  Serial.println("Probability of appearing after element /'a/'");
//  for (int i = 0; i < 6; i++){
//    Serial.print(elements[i] + ": ");
//    Serial.print(probs[i]);
//    Serial.println();
//  }

void trigNote(float freq, int atk, int sus, int rel){
  digitalWrite(LED_1, HIGH);
  envTimes[0] = atk;
  envTimes[1] = sus;
  envTimes[2] = rel;
  outFreq = freq;
  pointerInc = TABLESIZE * (outFreq / samplingRate);
  noteActive = true;
  noteEnd = millis() + envTimes[0] + envTimes[1] + envTimes[2];
  sectionStart = millis();
  sectionEnd = sectionStart + envTimes[0];
  envState = 0;
}

void setup() {
  //initialize GPIO pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_4, OUTPUT);
  
  analogReadSetup(inputPin);
  Timer1.initialize(1000000 / samplingRate);
  Timer1.attachInterrupt(analogReadStart);
  Serial.begin(115200);
//  trainMarkov();

    // calculate sine wavetable
  float RadAngle;                           // Angle in Radians
  for(int MyAngle=0;MyAngle<TABLESIZE;MyAngle++) {
    RadAngle=MyAngle*(2*PI)/TABLESIZE;               // angle converted to radians
    SineValues[MyAngle]=(sin(RadAngle)*127)+128;  // get the sine of this angle and 'shift' to center around the middle of output voltage range
  }

  //
  pointerInc = TABLESIZE * (freqs[outInd] / samplingRate);
  pointerVal = map(0, 0, TWO_PI, 0, TABLESIZE - 1);

//starts looking for C4
  goertzel.init(261.63);
  delay(1000);
}

void loop() {
  {
    int_guard guard {};
    goertzel.from(adcBuffer);
  }
  
  // try to detect some frequencies
//  // C4
//  goertzel.updateFreq(freqs[0]); //261.63
//  mags[0] = (goertzel.processAll() * 10);
//  // D4
//  goertzel.updateFreq(freqs[1]); //293.66
//  mags[1] = (goertzel.processAll() * 10);
//  // E4
//  goertzel.updateFreq(freqs[2]); //329.63
//  mags[2] = (goertzel.processAll() * 10);
//  // G4
//  goertzel.updateFreq(freqs[3]); //392.00
//  mags[3] = (goertzel.processAll() * 10);
//  // A4
//  goertzel.updateFreq(freqs[4]); //440.00
//  mags[4] = (goertzel.processAll(freqs[5]) * 10);
//  // C5
//  goertzel.updateFreq(freqs[5]); //523.25
//  mags[5] = (goertzel.processAll() * 10);
  

  //determine maximum magnitudes over listening period
  if (listening){
  for (int i = 0; i < 6; i++){
    goertzel.updateFreq(freqs[i]);
     mags[i] = (goertzel.processAll() * 10);
    if (mags[i] > magsMax[i]) magsMax[i] = mags[i];
  }
  }
  //print out magnitudes of all frequencies tested
//  Serial.print("C4: ");
//  Serial.print(mags[0]);
//  Serial.print(" D4: ");
//  Serial.print(mags[1]);
//  Serial.print(" E4: ");
//  Serial.print(mags[2]);
//  Serial.print(" G4: ");
//  Serial.print(mags[3]);
//  Serial.print(" A4: ");
//  Serial.print(mags[4]);
//  Serial.print(" C5: ");
//  Serial.println(mags[5]);


  //determine what note to play
  if( millis() > listenTimer && !noteActive){
    float chosenFreq = 0;
    //find max value of array and its corresponding frequency index
    float maxVal = magsMax[0];
    byte maxInd = 0;
    for (int i = 0; i < (sizeof(magsMax) / sizeof(magsMax[0])); i++) {
      if (magsMax[i] > maxVal){
        maxVal = magsMax[i];
        maxInd = i;
      }
    }
    //if max value is above a threshold, play a note
    if (maxVal > 4000){
//      chosenFreq = freqs[maxInd] * 1.0595; //one semitone up from played note
      chosenFreq = freqs[(maxInd+1)%((sizeof(magsMax) / sizeof(magsMax[0])))];
      trigNote(chosenFreq, 500, 1000, 500);
      listening = false;
      listenCount = 0;
    } else if(listenCount > 3){
      //play random note if it hasnt sang in a while
      byte randInd = random((sizeof(magsMax) / sizeof(magsMax[0])));
      trigNote(freqs[randInd], 500, 1000, 500);
      listenCount = 0;
      listening = false;
    }
    else {
      //don'tt play a note
      listenTimer += listenLength;
      listenCount++;
      listening = true;
    }
    for (int i = 0; i < (sizeof(magsMax) / sizeof(magsMax[0])); i++) {
      magsMax[i] = 0;
    }
    
  }

 
//  if (millis() > nextNote){
//    trigNote(200, 10, 50, 50);
//    nextNote = millis() + 3500;
//  }
//  bool toneDetected = magnitude > threshold;

//  printf("C4: %d, D4: %d, E4: %d, G4: %d, A4: %d, C5: %d\n", (int)(mags[0] * 100), (int)(mags[1] * 100), (int)(mags[2] * 100), (int)(mags[3] * 100), (int)(mags[4] * 100), (int)(mags[5] * 100)); 

//  once the tone is detected, play the next tone.
  
  //nextPitch();
 
// SWITCH DEPENDING ON THE BIRD
//  bool toneDetected = mags[0] > threshold;
//  digitalWrite(LED_BUILTIN, toneDetected);

  

  
//  printf("M=%03d |%c|%s>\n", (int)(magnitude), toneDetected ? '*' : ' ', bar);
  

  // Delay for a little bit
//  delay(300);
}
