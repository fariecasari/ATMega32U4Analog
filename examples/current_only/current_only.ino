// EmonLibDiff examples openenergymonitor.org, Licence GNU GPL V3

#include "EmonLibDiff.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance

void setup()
{  
  Serial.begin(9600);
  
  emon1.current(0, 10, 60.606);             // Current: input pin, gain, calibration for 33 Ohm.
}

void loop()
{
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only
  
  Serial.print(Irms*230.0);	       // Apparent power
  Serial.print(" ");
  Serial.println(Irms);		       // Irms
}
