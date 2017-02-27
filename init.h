#ifndef INITH
#define INITH

//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F12x
//-----------------------------------------------------------------------------

sfr16 RCAP2    = 0xca;                 // Timer2 capture/reload
sfr16 TMR2     = 0xcc;                 // Timer2

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define INJECTOR_MIN 1500
#define BAUDRATE     57600            // Baud rate of UART in bps

// SYSCLK = System clock frequency in Hz
#define SYSCLK       (22118400L * 9 / 4)

sbit AX1 = P1^6; 
sbit AX2 = P1^5;//P1^7;//servo 

sbit DX1 = P2^0;
sbit DX2 = P1^6;//P2^1;//starter
sbit DX3 = P1^7;//P2^2;//cooler

sbit BLINK = P7^0;

void UART0_Init (void);
void PORT_Init_UART0 (void);
void OSCILLATOR_Init (void);


#endif