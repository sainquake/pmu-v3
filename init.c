#include "c8051f120.h"
#include "init.h"



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



//	while (!(OSCXCN & 0x80));           // Wait for crystal osc to settle

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