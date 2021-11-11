/// Analog IO
///
/// Note: this library is different from the AsyncAnalog library
/// in the Arduino library manager!
///
/// To use this library, drag the AsyncADC.h and AsyncADC.cpp
/// files into your sketch folder.

#ifndef ASYNCADC_h
#define ASYNCADC_h

/// Call this function in setup()
///
/// This tells the ADC to select the correct analog pin later.
/// Pin number should be 0 for A0, 1 for A1, etc.
void analogReadSetup(uint8_t pin);

/// Tells the ADC to start measuring.
///
/// This function is asynchronous, meaning that the function
/// returns immediately so your program and do other things
/// while the ADC is busy taking in values.
///
/// When the ADC is done, your `ISR(ADC_vect)` subrountine will
/// be called.
void analogReadStart();

/// Read the current status of the ADC (a.k.a. if a conversion
/// is in progress).
int analogReadStatus();

/// Retrieve the last value.
///
/// You can safely call this function inside the ADC_vect
/// interrupt handler.
int analogReadGetValue();

#endif
