#include <c8051f120.h>                 // SFR declarations
#include <stdio.h>                     

#include <math.h>
#include <intrins.h>
#include <float.h>

#include "ADC0.h"
#include "PCA0.h"
//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F12x
//-----------------------------------------------------------------------------

sfr16 RCAP2    = 0xca;                 // Timer2 capture/reload
sfr16 TMR2     = 0xcc;                 // Timer2

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define PHASE_FRQ 0.005
#define BAUDRATE     57600//115200            // Baud rate of UART in bps

// SYSCLK = System clock frequency in Hz
#define SYSCLK       (22118400L * 9 / 4)
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void OSCILLATOR_Init (void);         
void PORT_Init_UART0 (void);
void UART0_Init (void);

xdata float bat3_pr;
xdata float bat;							// Заряд каждой банки
xdata float cx[4];
xdata float smoothCurrent;
xdata float cap;
xdata char SteckPoint;

xdata char phase = 0;
xdata char CountRun=0;

#define NBFM 		50
xdata char BuferFromModem [NBFM]; // Для анализа с последовательного порта
xdata char wBFM = 0, rBFM = 0, marBFM = 0;

#define SIZE_BUFFER0	61
xdata char BufferInModem[SIZE_BUFFER0]; // Для отправки в последовательный порт
xdata int r0, rk;

bit flTransmiter, flRun, flMem;
xdata char rgAnswer;
xdata char rst_src, rst_count=0;

void OutModem1(unsigned char Data, char i);
void OutModem2(unsigned int Data, char i);

void main (void)
{
	xdata char RK_code[66], nByte = 0, KontrSumma = 0, NPackage = 0;
	xdata int i;

	SFRPAGE = 0x00;
	rst_src = RSTSRC;
	rst_count++;

	SFRPAGE = CONFIG_PAGE;

	WDTCN = 0xde;                       // Disable watchdog timer
	WDTCN = 0xad;

	OSCILLATOR_Init ();                 // Initialize oscillator
	PORT_Init_UART0 ();                 // Initialize crossbar and GPIO

	UART0_Init ();                      // Initialize UART0
	PORT_Init_UART0 ();
	PCA0_Init ();
	ADC0_Init();

	SFRPAGE = 0x00;
	IE = 0x90;
	IP = 0x10;          //Interrupt Priority
	EIP1 = 0;        //Extended Interrupt Priority 1           Приоритет SPI = ?
	EIP2 = 0;        //Extended Interrupt Priority 2
	EA = 1;
	if(rst_src == 0x08) //если сброс от собаки   
	{
		;
	}
	else
	{
		flTransmiter = 0;
		flMem = 0;
		bat =0;
		for(i = 0; i < 4; i++)
		cx[i] = 0;
		smoothCurrent = cap = 0;
		bat3_pr = 0;
	}
	//Watchdog Enable
	WDTCN = 0x07;	   // Макс время = 0,021 с
	WDTCN = 0xA5;

	while (1)
	{
		CountRun = 0;

		//-------------RX Buffer Routein-------------//
		if(rBFM < (wBFM+marBFM*NBFM))
		{
			if ((BuferFromModem[rBFM] & 0xC0) == 0x40)	
			{
				nByte = 0;
				KontrSumma = 0;
				NPackage = BuferFromModem[rBFM] & 0x3f;
			}


			RK_code[nByte] = BuferFromModem[rBFM] & 0x7f;
			KontrSumma = KontrSumma^RK_code[nByte];
			if (++nByte > 65)
			nByte = 65;

			if ( (nByte == 3) && (KontrSumma == 0) )
			{
				if (NPackage == 1)
				{
					rgAnswer = 1;
				}
			}
			rBFM++;
			if(rBFM >= NBFM)
			{
				rBFM = 0;
				marBFM = 0;	
			}
		}

		//-------------TX Buffer Routein-------------
		if(flRun)
		{
			flRun = 0;
		}
		//rgAnswer = 1;
		if(flTransmiter)
		;
		else
		{
			if(rgAnswer == 1)
			{
				rgAnswer = 0;	

				BufferInModem[0] = 1 | 0x40;
				OutModem2( 100*bat, 1);
				OutModem2( 10*smoothCurrent, 3);
				OutModem2( 10*cap, 5);
				OutModem1( 4 | (char)flMem , 7);
				OutModem1( rst_src , 8);
				OutModem1( rst_count , 9);

				BufferInModem[10] = 0;
				for (i = 0; i < 10; i++ )
				BufferInModem[10] = BufferInModem[10] ^ BufferInModem[i];
				OutModem1(BufferInModem[10], 10);
				BufferInModem[11] = 0;
				r0 = 0;
				rk = 11;
				
				flTransmiter = 1;
				SFRPAGE = UART0_PAGE;
				TI0 = 1;
			} 
		}

	}//while(1)
}
//------------------------------------------------------------------------------
void OutModem1(unsigned char Data, char i)
{
	BufferInModem[i] = Data | 0x80;
}
//------------------------------------------------------------------------------
void OutModem2(unsigned int Data, char i)
{
	BufferInModem[i] = (Data & 0x007f)| 0x80;
	BufferInModem[i+1] = ((Data & 0x3f80) >> 7)| 0x80;
}
//------------------------------------------------------------------------------
void UART0_isr(void) interrupt 4
{
	xdata char SFRPAGE_SAVE = SFRPAGE;

	if (SteckPoint < SP)	SteckPoint = SP;

	SFRPAGE = UART0_PAGE;

	if (RI0)  //-------RX Get Byte-------//
	{ 
		BuferFromModem [wBFM++] = SBUF0;
		if(wBFM >= NBFM)
		{
			wBFM = 0;
			marBFM = 1;	
		}    
		RI0 = 0;		
	}
	if (TI0)  //-------TX Set Byte-------//
	{
		if(r0 < rk)
		SBUF0 = BufferInModem[r0++];
		else
		flTransmiter = 0;
		TI0 = 0;
	}
	SFRPAGE = SFRPAGE_SAVE;
	return;
}
//-----------------------------------------------------------------------------
// OSCILLATOR_Init
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{
	int i;                              // Software timer

	char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

	SFRPAGE = CONFIG_PAGE;              // Set SFR page

	OSCICN = 0x80;                      // Set internal oscillator to run
	// at its slowest frequency

	CLKSEL = 0x00;                      // Select the internal osc. as
	// the SYSCLK source

	// Initialize external crystal oscillator to use 22.1184 MHz crystal

	OSCXCN = 0x67;                      // Enable external crystal osc.
	for (i=0; i < 256; i++);            // Wait at least 1ms

	while (!(OSCXCN & 0x80));           // Wait for crystal osc to settle

	SFRPAGE = LEGACY_PAGE;
	FLSCL |=  0x30;                     // Initially set FLASH read timing for
	// 100MHz SYSCLK (most conservative
	// setting)
	if (SYSCLK <= 25000000) {           // Set FLASH read timing for <=25MHz
		FLSCL &= ~0x30;
	} else if (SYSCLK <= 50000000) {    // Set FLASH read timing for <=50MHz
		FLSCL &= ~0x20;
	} else if (SYSCLK <= 75000000) {    // Set FLASH read timing for <=75MHz
		FLSCL &= ~0x10;
	} else {                            // set FLASH read timing for <=100MHz
		FLSCL &= ~0x00;
	}

	// Start PLL for 50MHz operation
	SFRPAGE = PLL0_PAGE;
	PLL0CN = 0x04;                      // Select EXTOSC as clk source
	PLL0CN |= 0x01;                     // Enable PLL power
	PLL0DIV = 0x04;                     // Divide by 4
	PLL0FLT &= ~0x0f;
	PLL0FLT |=  0x0f;                   // Set Loop Filt for (22/4)MHz input clock
	PLL0FLT &= ~0x30;                   // Set ICO for 30-60MHz
	PLL0FLT |=  0x10;

	PLL0MUL = 0x09;                     // Multiply by 9

	// wait at least 5us
	for (i = 0; i < 256; i++) ;

	PLL0CN |= 0x02;                     // Enable PLL

	while (PLL0CN & 0x10 == 0x00);      // Wait for PLL to lock

	SFRPAGE = CONFIG_PAGE;

	CLKSEL = 0x02;                      // Select PLL as SYSCLK source

	SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}
//-----------------------------------------------------------------------------
// PORT_Init_UART0
//-----------------------------------------------------------------------------
void PORT_Init_UART0 (void)
{
	char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

	SFRPAGE = CONFIG_PAGE;              // Set SFR page

	XBR0     = 0x04;                    // Enable UART0
	XBR1     = 0x00;
	XBR2     = 0x40;                    // Enable crossbar and weak pull-up
	
	P0MDOUT |= 0x01;                    // Set TX pin to push-pull
	P1MDOUT |= 0xc0;                    // Set P1.6(LED) to push-pull
	P2MDOUT |= 0x07;

	SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//-----------------------------------------------------------------------------
// UART0_Init   Variable baud rate, Timer 2, 8-N-1
//-----------------------------------------------------------------------------
void UART0_Init (void)
{
	char SFRPAGE_SAVE;

	SFRPAGE_SAVE = SFRPAGE;             // Preserve SFRPAGE

	SFRPAGE = TMR2_PAGE;

	TMR2CN = 0x00;                      // Timer in 16-bit auto-reload up timer
	// mode
	TMR2CF = 0x08;                      // SYSCLK is time base; no output;
	// up count only
	RCAP2 = - ((long) SYSCLK/BAUDRATE/16);
	TMR2 = RCAP2;
	TR2= 1;                             // Start Timer2

	SFRPAGE = UART0_PAGE;

	SCON0 = 0x50;                       // 8-bit variable baud rate;
	// 9th bit ignored; RX enabled
	// clear all flags
	SSTA0 = 0x15;                       // Clear all flags; enable baud rate
	// doubler (not relevant for these
	// timers);
	// Use Timer2 as RX and TX baud rate
	// source;
	TI0     = 1;                        // Indicate TX0 ready

	SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}

//-------PCA Interrupt 400 Hz
void PCA0_ISR (void) interrupt 9
{
	xdata char SFRPAGE_SAVE= SFRPAGE;
	xdata unsigned int tmp;
	xdata float current;

	EA = 0;
	SFRPAGE = PCA0_PAGE;
	
	if(CF)
	{
		CF = 0;
		PCA0CN &= ~0x86;
		phase = !phase;
	}
	if (CCF0)                           // If Module 0 caused the interrupt
	{
		CCF0 = 0;                        // Clear module 0 interrupt flag.
	}
	if (CCF1)                           // If Module 0 caused the interrupt
	{
		CCF1 = 0;                        // Clear module 0 interrupt flag.
	}
	if (CCF2)                           // If Module 0 caused the interrupt
	{
		CCF2 = 0;                        // Clear module 0 interrupt flag.
	}
	if (CCF3)                           // If Module 0 caused the interrupt
	{
		CCF3 = 0;                        // Clear module 0 interrupt flag.
	}

	EA=1;
	if (SteckPoint < SP)	SteckPoint = SP;
	//--------200Hz sampling-----------//
	//flRun = 1;
	if(phase==0){
		tmp = 0;//analogRead(2);//2
		cx[3] = tmp*3120.0/4095.0;//*17.1/1.1*28500/5900/1000;

		tmp = 0;//analogRead(3);
		cx[2] = tmp*3120.0/4095.0;

		tmp = analogRead(1);
		cx[1] = tmp*3120.0/4095.0;

		tmp = analogRead(0);
		cx[0] = tmp*3120.0/4095.0;
		
		current =	(cx[0]+cx[1]+cx[2]+cx[3])*30000.0/820.0/1000.0;//*0.3758-17.395;//+cx[2]+cx[3]
		if(current<0)
			current = 0;
		smoothCurrent = ( smoothCurrent*9.0 + current )/10.0;
		cap += current*PHASE_FRQ/3600.0;

		tmp = analogRead(7);
		if(bat != bat3_pr)
			flMem = 1;
		bat = ( bat*9 + 25.0/25.8*tmp*3120.0/4095.0*(3.0+22.0)/3.0/1000.0 )/10.0;
		bat3_pr = bat;
		flRun = 1;
	}

	if (CountRun++ < 50)  
	WDTCN = 0xA5;	//Перезапустить охранный таймер
	SFRPAGE = SFRPAGE_SAVE;

}