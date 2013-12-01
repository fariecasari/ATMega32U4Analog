// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include "EmonLibDiff.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance

byte gain=1;

void setup()
{  
  Serial.begin(9600);
}

void loop()
{
  emon1.current(0, gain, 60.606);             // Current: input pin, gain, calibration for 33 Ohm.
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only
  
  Serial.print(Irms*230.0);	       // Apparent power
  Serial.print(" ");
  Serial.print(Irms);		       // Irms
  Serial.print(" x");
  Serial.println(gain);
  
  if(Irms > 10) gain=1;
  else if(Irms > 2.5) gain=10;
  else if(Irms > 0.5) gain=40;
  else gain=200;
}
