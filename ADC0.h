#ifndef _ADC0_H
#define _ADC0_H

#include <c8051f120.h>
#include <intrins.h>

#define ANALOG_INPUTS 8                // Number of AIN pins to measure 
sfr16 ADC0     = 0xbe;                 // ADC0 data

void ADC0_Init (void);
unsigned int analogRead(unsigned char ch);

#endif