#include "TimerOne.h"
#include "AsyncAnalog.h"
#include <SPI.h>
#include <DAC_MCP49xx.h>
#define SS_PIN 10 //define chip select pin
#define TABLESIZE 50
#include <MarkovChain.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;


#define LED_1 PD5
#define LED_2 PD6
#define BUTTON_PIN PD7
#define LED_4 PB0

#define printf(fmt, ...) \
  snprintf(_printfBuffer, 64, fmt, ##__VA_ARGS__);\
  Serial.print(_printfBuffer);
  
  // initialize possible nums with length 100 and 0s in all inds
  char possibleNotes[100];
  
int SineValues[TABLESIZE]; 
float pointerInc;
float pointerVal = 0;
int outVal;

int outInd = 0;
int curSection = 0;
float freqs [4][5] = {{261.63, 293.66, 329.63, 392.00, 440.00},
                  {329.63, 369.99, 392.00, 493.88, 523.25},
                  {207.65, 261.63, 311.13, 349.23, 369.99},
                  {466.16, 523.25, 587.33, 698.46, 783.99}};
float leaderFreq = 1108.73;
float leaderMag = 0;

// defining the possible current frequencies with curSection - curSection must be updated every time it hears the leader freq
float curFreqs [5];
float curFreqsPlusLeader[6];

char curGlobalNote = 'a';
// each letter corresponds to a frequency, not the actual note name just placeholders for it to be human readable later.
char states[] = {'a','b','c','d','e'};

// important note: each row and each column must sum to 1 
// AND must not go over 2 points after the decimal. 
// you are doing sudoku
Matrix<5,5> tMatrix = {0.2,  0.25,  0.25, 0.1,  0.2,
                       0.2,  0.1,   0.2,  0.3,  0.2,
                       0.2,  0.25,  0.25, 0.3,  0,
                       0.3,  0.4,   0,    0.3,  0,
                       0.1,  0,     0.3,  0,    0.6};


// attack, sustain, and release global variables that can be changed based on how loud things are around it. setting defaults here
float attackGlobal = 500;
float sustainGlobal = 1000;
float releaseGlobal = 500;

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
float maxVal;

//note clock
unsigned long nextNote = 0;

//note control
boolean listening = true;
unsigned long listenTimer = 2000;
unsigned long listenTimer2 = 2000;

unsigned long listenLength = 2000;
byte listenCount = 0;
float outFreq = 100.00;

boolean leaderTimeout = false;
unsigned long leaderTimer = 10000;

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
    Serial.print(targetFreq);
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

// takes in magnitudes from magsMax which is a global variable and returns a, s, and r values
void densityOfNotes(){
  float totalLoud = 0;
  for(int i=0; i<(sizeof(magsMax)/sizeof(magsMax[0])); i++){
    totalLoud = totalLoud + magsMax[i];
  }
  // the higher the totalLoud, the smaller the a, s, and r values are. maybe
  // we are assuming 4000 is a medium loud sound.

  // default attack: 500
  // default sustain: 1000
  // default release: 500
  // if it gets 4000 the default should have a multiplier of 1
  
  float mediumLoud = 4000; 

  // maps to -500x/(1.5 * 4000) + 1000 so it crosses 0 at 12000. hopefully totalLoud doesnt get that loud.
  if (totalLoud < 11000){
      attackGlobal = 500 * (-1)* totalLoud / (mediumLoud * 1.5) + 1000;
  sustainGlobal = 1000* (-1) * totalLoud/(mediumLoud) + 1500;
  releaseGlobal = 500 * (-1)* totalLoud / (mediumLoud * 1.5) + 1000;
  }
  else{
    attackGlobal = 83;
    sustainGlobal = 166;
    releaseGlobal = 83;
  }
}


void cycle(){
  pointerInc = TABLESIZE * (curFreqs[outInd] / samplingRate);
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
    envVal = 0;
      noteActive = 0;
      listening = true;
      listenTimer = millis() + listenLength;
      digitalWrite(LED_1, LOW);
      for (int i=0; i<(sizeof(mags)/sizeof(mags[0])); i++){
        mags[i] = 0;
        magsMax[i] = 0;
      }
      maxVal = 0;
      delay(750);
      
    break;
  }
  //multiply by envelope value
  outVal *= envVal;
  //increment wavetable pointer
  pointerVal += pointerInc;
  if(pointerVal > TABLESIZE) pointerVal -= TABLESIZE;
}


// markov takes in current value and returns next value. hardcoded with 5 notes as the total number of notes
// using the algorithm given by Jongware in https://stackoverflow.com/questions/20327958/random-number-with-probabilities
char markov(char curVal) {
//  find the current value's index in the states[] array
  int wantedpos = -1;
  for (int i=0; i<(sizeof(states)/sizeof(states[0])); i++) {
    if (curVal == states[i]) {
      wantedpos = i;
      break;
    }
  }
  
  // put a 1 in the current position so we can multiply this matrix by the probability matrix
  Matrix<5,1> curDist = {0,0,0,0,0};
  curDist(wantedpos) = 1;

  Matrix<5,1> outputChances = tMatrix * curDist;

  // the rest of this used to be trainMarkov and it is only called within the markov this should work
  int randInd = random(100);
    
  // make an array with all the possible numbers that it could be. length 100
  int curIndex=0;
  for (int i=0; i<outputChances.Rows; i++){
    int currentChance = outputChances(i) * 100;

    char currentState = states[i];
    
    for (int j=0; j<currentChance; j++){\
      // fill possibleNotes with the currentState currentChance number of times
      possibleNotes[curIndex] = currentState;
      curIndex++;
    }
  }
  // so now possibleNums should be 100 numbers long with "a","b","c","d","e" tons of times. see which index the randInd falls on.
  char nextNote = possibleNotes[randInd];
  
  Serial.print(nextNote);
  Serial.println();

  return nextNote;  
}


void trigNote(float freq, int atk, int sus, int rel){
  Serial.print("playing a note: ");
  Serial.print(freq);
  Serial.println();
    for (int i = 0; i < (sizeof(magsMax) / sizeof(magsMax[0])); i++) {
      magsMax[i] = 0;
      mags[i] = 0;
    }
    maxVal = 0;
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

void changeSectionFromLeader(){
  curSection = (curSection + 1) % 4;
  for (int i=0; i<(sizeof(curFreqs)/sizeof(curFreqs[0])); i++){
    curFreqs[i] = freqs[curSection][i];
  }
  Serial.print("i heard the leader");
  Serial.println();
}


void setup() {
  //initialize GPIO pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_4, OUTPUT);
  digitalWrite(LED_2,HIGH);
  
  analogReadSetup(inputPin);
  Timer1.initialize(1000000 / samplingRate);
  Timer1.attachInterrupt(analogReadStart);
  Serial.begin(115200);

  
  for (int i=0; i<6; i++){
    if (i<5){
      curFreqs[i] = freqs[curSection][i];
      curFreqsPlusLeader[i] = curFreqs[i];
    }
    else{
      curFreqsPlusLeader[i] = leaderFreq;
    }
  }

    // calculate sine wavetable
  float RadAngle;                           // Angle in Radians
  for(int MyAngle=0;MyAngle<TABLESIZE;MyAngle++) {
    RadAngle=MyAngle*(2*PI)/TABLESIZE;               // angle converted to radians
    SineValues[MyAngle]=(sin(RadAngle)*127)+128;  // get the sine of this angle and 'shift' to center around the middle of output voltage range
  }

  pointerInc = TABLESIZE * (curFreqs[outInd] / samplingRate);
  pointerVal = map(0, 0, TWO_PI, 0, TABLESIZE - 1);


  trigNote(100,500,1000,500);

  
//starts looking for C4
  goertzel.init(261.63);
  delay(1000);
}

void loop() {
  {
    int_guard guard {};
    goertzel.from(adcBuffer);
  }

  Serial.print(listening);
  Serial.println();
  
      goertzel.updateFreq(curFreqsPlusLeader[5]);
      leaderMag = (goertzel.processAll() * 10);
        if (leaderMag > 2000 && millis() > leaderTimer){
          changeSectionFromLeader();
          leaderTimer = millis() + 10000;
        }
  
  //determine maximum magnitudes over listening period
  if (listening){
    for (int i = 0; i < (sizeof(curFreqsPlusLeader) / sizeof(curFreqsPlusLeader[0])); i++){
      goertzel.updateFreq(curFreqsPlusLeader[i]);
      mags[i] = (goertzel.processAll() * 10);
      if (mags[i] > magsMax[i]) {
        magsMax[i] = mags[i];
      }
    }
  }
  //print out magnitudes of all frequencies tested
//  Serial.print("C4: ");
//  Serial.print(magsMax[0]);
//  Serial.print(" D4: ");
//  Serial.print(magsMax[1]);
//  Serial.print(" E4: ");
//  Serial.print(magsMax[2]);
//  Serial.print(" G4: ");
//  Serial.print(magsMax[3]);
//  Serial.print(" A4: ");
//  Serial.print(magsMax[4]);
//  Serial.print(" C5: ");
//  Serial.println(magsMax[5]);
  


  //determine what note to play  
//  Serial.println(listenTimer);
//  Serial.println(millis());
  
  if( millis() > (listenTimer) && !noteActive){
      Serial.print("C4: ");
  Serial.print(magsMax[0]);
  Serial.print(" D4: ");
  Serial.print(magsMax[1]);
  Serial.print(" E4: ");
  Serial.print(magsMax[2]);
  Serial.print(" G4: ");
  Serial.print(magsMax[3]);
  Serial.print(" A4: ");
  Serial.print(magsMax[4]);
  Serial.print(" C5: ");
  Serial.println(magsMax[5]);
    float chosenFreq = 0;
    //find max value of array and its corresponding frequency index
    maxVal = magsMax[0];
    byte maxInd = 0;
    for (int i = 0; i < (sizeof(magsMax) / sizeof(magsMax[0])); i++) {
      if (magsMax[i] > maxVal){
        maxVal = magsMax[i];
        maxInd = i;
      }
    }
    //if max value is above a threshold, play a note
    if (maxVal > 4000){
      Serial.println(maxVal);
      nextNote = markov(curGlobalNote);
      curGlobalNote = nextNote;
      // decide what index of the frequency values array the letter corresponds to
      int curNoteInd;
      for (int i=0; i<(sizeof(states)/sizeof(states[0])); i++) {
        if (curGlobalNote == states[i]) {
          curNoteInd = i;
          break;
        }
      }
      chosenFreq = curFreqs[curNoteInd];

      //create the right asr values
//      densityOfNotes();
//      trigNote(chosenFreq, attackGlobal, sustainGlobal, releaseGlobal);
      trigNote(chosenFreq, 500, 500, 500);
      listening = false;
      listenCount = 0;
    }
    else {
      //don't play a note
      listenTimer += listenLength;
      listenCount++;
      listening = true;
    }
    for (int i = 0; i < (sizeof(magsMax) / sizeof(magsMax[0])); i++) {
      magsMax[i] = 0;
      mags[i] = 0;
    }
    maxVal = 0;
  }
}
