#include <c8051f120.h>                 // SFR declarations
#include <stdio.h>                     

#include <math.h>
#include <intrins.h>
#include <float.h>

#include "ADC0.h"
#include "PCA0.h"
#include "init.h"
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
xdata float bat;
xdata float minbat=50;							// Заряд каждой банки
xdata float cx[4];
xdata float smoothCurrent;
xdata float cap;
xdata char SteckPoint;
xdata unsigned int time;

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

	SFRPAGE = 0x0f;
	BLINK = 0;
	SFRPAGE = 0x00;

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
		time=0;
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
				//	BLINK = 1;
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
				OutModem2( 100.0*bat, 1);
				OutModem2( 10.0*smoothCurrent, 3);
				OutModem2( 10.0*cap, 5);
				OutModem1( 4.0*minbat, 7);//4 | (char)flMem
				OutModem1( rst_src , 8);
				OutModem1( rst_count , 9);

				BufferInModem[10] = 0;
				for (i = 0; i < 10; i++ )
				BufferInModem[10] = BufferInModem[10] ^ BufferInModem[i];
				OutModem1(BufferInModem[10], 10);
				BufferInModem[11] = 0;
				r0 = 0;
				rk = 11;
				
				minbat = 50;
				flTransmiter = 1;
				SFRPAGE = UART0_PAGE;
				TI0 = 1;
			//	BLINK = 0;
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
//-------PCA Interrupt 400 Hz
void PCA0_ISR (void) interrupt 9
{
	xdata char SFRPAGE_SAVE= SFRPAGE;
	xdata unsigned int tmp;
	xdata float current,ftmp;

	EA = 0;
	SFRPAGE = PCA0_PAGE;
	
	if(CF)
	{
		CF = 0;
		PCA0CN &= ~0x86;
		phase = !phase;

			PCA0L = (PCA0_OFFSET & 0x00FF);//регистр таймера
			PCA0H = (PCA0_OFFSET & 0xFF00)>>8;

			PCA0CPL0 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)100)) & 0x00FF);//регистр сравнения
			PCA0CPH0 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)0)) & 0xFF00)>>8;

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
		tmp = analogRead(2);//2
		cx[3] = tmp*3120.0/4095.0;//*17.1/1.1*28500/5900/1000;

		tmp = analogRead(3);
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

		tmp = analogRead(4);//old num 7
		if(bat != bat3_pr)
			flMem = 1;
		ftmp = 25.0/25.8*tmp*3120.0/4095.0*(3.0+22.0)/3.0/1000.0;
		bat = ( bat*49 + ftmp )/50.0;
		if(ftmp<minbat)
			minbat = ftmp;
		bat3_pr = bat;
		flRun = 1;
		
		SFRPAGE = 0x0f;
		time++;
		if(time>100)
			BLINK = 1;
		else 
			BLINK = 0;
		if(time>200){
			BLINK = 0;
			time=0;
		}
		//BLINK=1;
		SFRPAGE = PCA0_PAGE;
	}
	if(phase==1){
		/*SFRPAGE = 0x0f;
		BLINK=0;
		SFRPAGE = PCA0_PAGE;*/
	}
	if (CountRun++ < 50)  
	WDTCN = 0xA5;	//Перезапустить охранный таймер
	SFRPAGE = SFRPAGE_SAVE;

}