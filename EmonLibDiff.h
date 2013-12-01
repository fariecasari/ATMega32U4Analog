/*
  Emon.h - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
	Modified for differential ADC mode by Martin Roberts, July 2 2013
  GNU GPL
*/

#ifndef EmonLibDiff_h
#define EmonLibDiff_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

class EnergyMonitor
{
  public:

    void voltage(int _inPinV, double _VCAL, double _PHASECAL); // _inPinV can be 0-3 or 5
    void current(int _inPinI, int _inGainI, double _ICAL); // _inPinI can be 0-3, _inGainI can be 1,10,40 or 200

    void voltageTX(double _VCAL, double _PHASECAL);
    void currentTX(int _channel, int _inGainI, double _ICAL);

    void calcVI(int crossings, int timeout);
    double calcIrms(int NUMBER_OF_SAMPLES);
    void serialprint();

    //Useful value variables
    double realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;

private:

		int analogReadDiff(byte mux);

    //Set Voltage and current input mux selection
    int inMuxV;
    int inMuxI;
    //Calibration coeficients
    //These need to be set in order to obtain accurate results
    double VCAL;
    double ICAL;
    double PHASECAL;
		// amplifier gain
		int inGainI;

    //--------------------------------------------------------------------------------------
    // Variable declaration for emon_calc procedure
    //--------------------------------------------------------------------------------------
	int lastSampleV,sampleV;   //sample_ holds the raw analog read value, lastSample_ holds the last sample
	int sampleI;                      

	double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.

	long sqV,sumV,sqI,sumI;														//sq = squared, sum = Sum
	double instP,sumP;                                //inst = instantaneous

	int startV;                                       //Instantaneous voltage at start of sample window.

	boolean lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.
	int crossCount;                                   // ''

};

#endif
