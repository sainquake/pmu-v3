/*#include "c8051f120.h"
#include <stdio.h> 
#include <math.h>
#include <stdlib.h>
#include <intrins.h>
#include <float.h>*/
#include "ADC0.h"
//ADC


void ADC0_Init (void)
{
	SFRPAGE = 0x00;
	REF0CN = 0x03;	// Reference Control Register
	AMX0CF = 0x00;	// AMUX Configuration Register
	ADC0CF = 0x80;
	ADC0CN = 0x80;

	SFRPAGE = 0x02;
	AMX2CF = 0x00;	// AMUX Configuration Register
	ADC2CF = 0x81;
	ADC2CN = 0x80;

	ADC2LT = 0x00;	// ADC Less-Than High Byte Register
	ADC2GT = 0xFF;	// ADC Greater-Than High Byte Register
}
unsigned int analogRead(unsigned char ch)
{
	xdata int uitmp;
	xdata char SFRPAGE_SAVE = SFRPAGE; 
	SFRPAGE = ADC0_PAGE;
	
		SFRPAGE = 0x00;
			AMX0SL = ch;	
			for (uitmp = 0; uitmp < 10; uitmp++)
				_nop_();
	     	AD0INT = 0;
			AD0BUSY = 1;
     		AD0INT = 0;
			while(AD0BUSY);
	SFRPAGE = SFRPAGE_SAVE;

	return (ADC0 & 0x0fff);
}