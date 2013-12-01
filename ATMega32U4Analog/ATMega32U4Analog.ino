/*
wiring_differential.c — analog differential input
by Muh Nahdhi Ahsan http://sekarlangit.com/arduino-differential-gain.php

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General
Public License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330,
Boston, MA 02111-1307 USA

wiring_differential.c modified from wiring_analog.c
wiring_analog.c — Copyright (c) 2005-2006 David A. Mellis
wiring_analog.c — modified 28 September 2010 by Mark Sproul
wiring_analog.c — Part of Arduino — http://arduino.cc

$Id: wiring.c 248 2007-02-03 15:36:30Z mellis $
*/

#include «wiring_private.h»
#include «pins_arduino.h»

uint8_t analog_reference = DEFAULT;

void analogReference(uint8_t mode)
{
// can’t actually set the register here because the default setting
// will connect AVCC and the AREF pin, which would cause a short if
// there’s something connected to AREF.
analog_reference = mode;
}

// Чтение аналогового диф. сигнала без усиления (read datashet of ATMega1280 and ATMega2560 for refference)
// Use analogReadDiff(NUM)
// NUM | POS PIN | NEG PIN | GAIN
// 0 | A0 | A1 | 1x
// 1 | A1 | A1 | 1x
// 2 | A2 | A1 | 1x
// 3 | A3 | A1 | 1x
// 4 | A4 | A1 | 1x
// 5 | A5 | A1 | 1x
// 6 | A6 | A1 | 1x
// 7 | A7 | A1 | 1x
// 8 | A8 | A9 | 1x
// 9 | A9 | A9 | 1x
// 10 | A10 | A9 | 1x
// 11 | A11 | A9 | 1x
// 12 | A12 | A9 | 1x
// 13 | A13 | A9 | 1x
// 14 | A14 | A9 | 1x
// 15 | A15 | A9 | 1x

int analogReadDiff(uint8_t pin)
{
uint8_t low, high;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__)
if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(__AVR_ATmega32U4__)
pin = analogPinToChannel(pin);
ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0×01) << MUX5);
#elif defined(ADCSRB) && defined(MUX5)
// the MUX5 bit of ADCSRB selects whether we’re reading from channels
// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0×01) << MUX5);
#endif
// set the analog reference (high two bits of ADMUX) and select the
// channel (low 4 bits). this also sets ADLAR (left-adjust result)
// to 0 (the default).
#if defined(ADMUX)
ADMUX = (analog_reference << 6) | ((pin | 0×10) & 0x1F);
#endif

// without a delay, we seem to read from the wrong channel
//delay(1);

#if defined(ADCSRA) && defined(ADCL)
// start the conversion
sbi(ADCSRA, ADSC);

// ADSC is cleared when the conversion finishes
while (bit_is_set(ADCSRA, ADSC));

// we have to read ADCL first; doing so locks both ADCL
// and ADCH until ADCH is read. reading ADCL second would
// cause the results of each conversion to be discarded,
// as ADCL and ADCH would be locked when it completed.
low = ADCL;
high = ADCH;
#else
// we dont have an ADC, return 0
low = 0;
high = 0;
#endif

// combine the two bytes
return (high << 8) | low; } // Read Analog Differential with gain (read datashet of ATMega1280 and ATMega2560 for refference) // Use analogReadGain(NUM) // NUM | POS PIN | NEG PIN | GAIN // 0 | A0 | A0 | 10x // 1 | A1 | A0 | 10x // 2 | A0 | A0 | 200x // 3 | A1 | A0 | 200x // 4 | A2 | A2 | 10x // 5 | A3 | A2 | 10x // 6 | A2 | A2 | 200x // 7 | A3 | A2 | 200x // 8 | A8 | A8 | 10x // 9 | A9 | A8 | 10x // 10 | A8 | A8 | 200x // 11 | A9 | A8 | 200x // 12 | A10 | A10 | 10x // 13 | A11 | A10 | 10x // 14 | A10 | A10 | 200x // 15 | A11 | A10 | 200x int analogReadGain(uint8_t pin) { uint8_t low, high; #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__)
if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(__AVR_ATmega32U4__)
pin = analogPinToChannel(pin);
ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0×01) << MUX5);
#elif defined(ADCSRB) && defined(MUX5)
// the MUX5 bit of ADCSRB selects whether we’re reading from channels
// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0×01) << MUX5);
#endif
// set the analog reference (high two bits of ADMUX) and select the
// channel (low 4 bits). this also sets ADLAR (left-adjust result)
// to 0 (the default).
#if defined(ADMUX)
ADMUX = (analog_reference << 6) | ((pin | 0×08) & 0x0F);
#endif

// without a delay, we seem to read from the wrong channel
//delay(1);

#if defined(ADCSRA) && defined(ADCL)
// start the conversion
sbi(ADCSRA, ADSC);

// ADSC is cleared when the conversion finishes
while (bit_is_set(ADCSRA, ADSC));

// we have to read ADCL first; doing so locks both ADCL
// and ADCH until ADCH is read. reading ADCL second would
// cause the results of each conversion to be discarded,
// as ADCL and ADCH would be locked when it completed.
low = ADCL;
high = ADCH;
#else
// we dont have an ADC, return 0
low = 0;
high = 0;
#endif

// combine the two bytes
return (high << 8) | low;
}


void setup() {}

void loop() {}
