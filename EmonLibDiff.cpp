/*
  EmonLibDiff.cpp - Library for openenergymonitor using differential ADC mode
  Created by Trystan Lea, April 27 2010
	Modified for differential ADC mode by Martin Roberts, July 2 2013
  GNU GPL
*/

//#include "WProgram.h" un-comment for use on older versions of Arduino IDE
#include "EmonLibDiff.h"

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

#define REFVOLTAGE 2.56

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(int _inPinV, double _VCAL, double _PHASECAL)
{
	if(_inPinV == 5) inMuxV = 0; // A5 is on ADC0
  else inMuxV = (7 - _inPinV); // A0-A3 are on ADC7-ADC4
	inMuxV |= 0x10;// -ve differential i/p is ADC1
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
}

void EnergyMonitor::current(int _inPinI, int _inGainI, double _ICAL)
{
	inMuxI = 7 - _inPinI; // A0-A3 are on ADC7-ADC4
	switch(_inGainI)
	{
		case 200: inMuxI |= 0x38; break; // -ve differential i/p is ADC1
		case  40: inMuxI |= 0x30; break;
		case  10: inMuxI |= 0x28; break;
		 default: inMuxI |= 0x10; break;
	}
	inGainI = _inGainI;
  ICAL = _ICAL;
}

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors based on emontx shield pin map
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltageTX(double _VCAL, double _PHASECAL)
{
	voltage(0, _VCAL, _PHASECAL);
}

void EnergyMonitor::currentTX(int _channel, int _inGainI, double _ICAL)
{
	current(_channel, _inGainI, _ICAL);
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kwh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void EnergyMonitor::calcVI(int crossings, int timeout)
{
  int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  int numberOfSamples = 0;                        //This is now incremented  

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  boolean st=false;                                  //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesn't get stuck in the loop if there is an error.

  while(st==false)                                   //the while loop...
  {
     startV = analogReadDiff(inMuxV);                    //using the voltage waveform
     if ((startV < 50) && (startV > -50)) st=true;  //check its within range
     if ((millis()-start)>timeout) st = true;
  }
  
  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurment loop
  //------------------------------------------------------------------------------------------------------------------------- 
  start = millis(); 

  while ((crossCount < crossings) && ((millis()-start)<timeout)) 
  {
    numberOfSamples++;                            //Count number of times looped.

    lastSampleV=sampleV;                          //Used for phase shift
    
    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = analogReadDiff(inMuxV);                 //Read in raw voltage signal
    sampleI = analogReadDiff(inMuxI);                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
    sqV= (long)sampleV * sampleV;                 //1) square voltage values
    sumV += sqV;                                //2) sum
    
    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------   
    sqI = (long)sampleI * sampleI;                //1) square current values
    sumI += sqI;                                //2) sum 
    
    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastSampleV + PHASECAL * (sampleV - lastSampleV); 
    
    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------   
    instP = phaseShiftedV * sampleI;          //Instantaneous Power
    sumP +=instP;                               //Sum  
    
    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------       
    lastVCross = checkVCross;                     
    if (sampleV > startV) checkVCross = true; 
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;                  
                     
    if (lastVCross != checkVCross) crossCount++;
  }
 
  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //------------------------------------------------------------------------------------------------------------------------- 
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coeficients applied. 
  
  double V_RATIO = VCAL * REFVOLTAGE / 512.0;
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples); 
  
  double I_RATIO = (ICAL * REFVOLTAGE / 512.0) / inGainI;
  Irms = I_RATIO * sqrt((double)sumI / numberOfSamples); 

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
//--------------------------------------------------------------------------------------       
}

//--------------------------------------------------------------------------------------
double EnergyMonitor::calcIrms(int NUMBER_OF_SAMPLES)
{
  for (int n = 0; n < NUMBER_OF_SAMPLES; n++)
  {
    sampleI = analogReadDiff(inMuxI);

    // Root-mean-square method current
    // 1) square current values
    sqI = (long)sampleI * sampleI;
    // 2) sum 
    sumI += sqI;
  }

  double I_RATIO = (ICAL * REFVOLTAGE / 512.0) / inGainI;
  Irms = I_RATIO * sqrt((double)sumI / NUMBER_OF_SAMPLES); 

  //Reset accumulators
  sumI = 0;
//--------------------------------------------------------------------------------------       
 
  return Irms;
}

void EnergyMonitor::serialprint()
{
    Serial.print(realPower);
    Serial.print(' ');
    Serial.print(apparentPower);
    Serial.print(' ');
    Serial.print(Vrms);
    Serial.print(' ');
    Serial.print(Irms);
    Serial.print(' ');
    Serial.print(powerFactor);
    Serial.println(' ');
    delay(100); 
}

// new differential analog read function for any input source & gain
int EnergyMonitor::analogReadDiff(byte mux)
{
	// 2.56V ref, left justified (for sign extension)
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(ADLAR) | (mux & 0x1f);
	ADCSRB = _BV(ADHSM); // high speed mode
	if(mux & 0x20) ADCSRB |= _BV(MUX5);
	delayMicroseconds(125); // required for amplifier to settle
	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS2) | _BV(ADPS1); // start conversion, 250kHz clock
  while(ADCSRA & _BV(ADSC)); // wait for completion
	return ((int)ADC)>>6; // right justify and sign extend
}
