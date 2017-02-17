#include <c8051f120.h>                 // SFR declarations
#include <stdio.h>                     

#include <math.h>
#include <stdlib.h>
#include <intrins.h>
#include <float.h>

#include "init.h"
#include "ADC0.h"
#include "PCA0.h"
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

xdata float bat;	
xdata float bat3_pr;
xdata float cx[4];
xdata float current;
xdata float smoothCurrent;
xdata float smoothCurrentMax;
xdata float Wh;

xdata float cap;
xdata char SteckPoint;
xdata float chgCurrent;
xdata float chgCurrSmooth;
xdata float maxChg;
xdata float temperature;

xdata float pwm3 = 1000;
xdata float pwm = 1000;
xdata float pwm1 = 1000;
xdata float pwm2 = 1000;
xdata float pwm4 = 700*PCA0_MKS;

xdata char startDvs;
xdata char phase = 0;
xdata char CountRun=0;

void OutModem1(unsigned char Data, char i);
void OutModem2(unsigned int Data, char i);
void OutModem4(unsigned long int Data, char i);

#define NBFM 		50
xdata char BuferFromModem [NBFM]; // Для анализа с последовательного порта
xdata char wBFM = 0, rBFM = 0, marBFM = 0;

#define SIZE_BUFFER0	61
xdata char BufferInModem[SIZE_BUFFER0]; // Для отправки в последовательный порт
xdata int r0, rk;

bit flTransmiter, flRun;
char flMem;
xdata char rgAnswer;
xdata char rst_src, rst_count=0;



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
	//EIP2 = 0x40;        //Extended Interrupt Priority 2
	EA = 1;
	if(rst_src == 0x08) //если сброс от собаки   
	{
		;
	}
	else
	{
		startDvs=0;
		flTransmiter = 0;
		flMem = 0;
		for (i = 0; i < 9; i++)
		bat =0;// cell[i] = 0;
		for(i = 0; i < 4; i++)
		cx[i] = 0;
		current = smoothCurrent = cap = smoothCurrentMax = 0;
		chgCurrent = 0;
		maxChg = 0;
		chgCurrSmooth=0;
	}

	//Watchdog Enable
	WDTCN = 0x07;	   // Макс время = 0,021 с
	WDTCN = 0xA5;

	/*pwm[0]=0;
	pwm[1]=0;
	pwm[2]=0;*/
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
				if (NPackage == 2)//start dvs
				{
					rgAnswer = 2;
					startDvs=1;
				}
				if (NPackage == 3)//stop dvs
				{
					rgAnswer = 3;
					startDvs=0;
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
		//	rgAnswer = 1;
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
//xdata float ttt=0;
//-------PCA Interrupt 400 Hz
void PCA0_ISR (void) interrupt 9
{
	xdata char SFRPAGE_SAVE= SFRPAGE;
	xdata int tmp;
	xdata float rtemp;

	EA = 0;

	SFRPAGE = PCA0_PAGE;
	
	if(CF)
	{
		CF = 0;
		PCA0CN &= ~0x86;
		phase = !phase;

		if(phase)
		{
			DX3=1;

			//if(pwm3>0)
			AX2 = 1;//pwm3

			if(pwm1>0)
			DX1 = 1;//pwm1

			if(pwm2>0)
			DX2 = 1;//pwm2


			//PCA0 = PCA0_OFFSET;
			PCA0L = (PCA0_OFFSET & 0x00FF);//регистр таймера
			PCA0H = (PCA0_OFFSET & 0xFF00)>>8;

			//PCA0CP0 = PCA0_OFFSET+PCA0_DEFINITION*pwm;
			/*txx = ttt/2*PCA0_MKS;
			if(txx>500*PCA0_MKS){
				txx = 500*PCA0_MKS;
			}*/
			//pwm3 = 1100*PCA0_MKS;

			PCA0CPL0 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)pwm3)) & 0x00FF);//регистр сравнения
			PCA0CPH0 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)pwm3)) & 0xFF00)>>8;

			//PCA0CP1 = PCA0_OFFSET+PCA0_DEFINITION*pwm1;
			PCA0CPL1 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)pwm1)) & 0x00FF);//регистр сравнения
			PCA0CPH1 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)pwm1)) & 0xFF00)>>8;
			
			//PCA0CP2 = PCA0_OFFSET+PCA0_DEFINITION*pwm2;
			PCA0CPL2 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)pwm2)) & 0x00FF);//регистр сравнения
			PCA0CPH2 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)pwm2)) & 0xFF00)>>8;

			PCA0CPL3 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)pwm4)) & 0x00FF);//регистр сравнения
			PCA0CPH3 = ((PCA0_OFFSET+PCA0_DEFINITION*((unsigned int)pwm4)) & 0xFF00)>>8;
		}
	}
	if (CCF0)                           // If Module 0 caused the interrupt
	{
		CCF0 = 0;                        // Clear module 0 interrupt flag.
		AX2 = 0;                      // Invert the LED pin
		
	}
	if (CCF1)                           // If Module 0 caused the interrupt
	{
		CCF1 = 0;                        // Clear module 0 interrupt flag.
		DX1 = 0;
	}
	if (CCF2)                           // If Module 0 caused the interrupt
	{
		CCF2 = 0;                        // Clear module 0 interrupt flag.
		DX2 = 0;
	}
	if (CCF3)                           // If Module 0 caused the interrupt
	{
		CCF3 = 0;                        // Clear module 0 interrupt flag.
		DX3 = 0;
	}

	EA=1;
	if (SteckPoint < SP)	SteckPoint = SP;

	//pwm[0] = pwm1 = pwm2 = 1000.0*PCA0_MKS;
	//pwm1 = pwm2 = 12270;//1ms

	//--------400Hz sampling-----------//
	//flRun = 1;
	if(phase==0){
		//measure voltage prop. to ICE current 
		tmp = analogRead(5);
		chgCurrent = (tmp*3120.0/4095-1664)/10;
		chgCurrSmooth = (chgCurrSmooth*19 + chgCurrent)/20;
		if(chgCurrent>maxChg)
			maxChg = chgCurrent;
		//measure voltage proportional to current from BTS555
		tmp = analogRead(2);//2
		cx[3] = tmp*3120.0/4095;//*17.1/1.1*28500/5900/1000;

		tmp = analogRead(3);
		cx[2] = tmp*3120.0/4095;

		tmp = analogRead(1);
		cx[1] = 0;//tmp*3120.0/4095;

		tmp = analogRead(0);
		cx[0] = tmp*3120.0/4095;
		//calc total current 
		current =	(cx[0]+cx[1]+cx[2]+cx[3])*30000.0/820.0/1000.0;//*0.3758-17.395;//+cx[2]+cx[3]
		if(current<0)
			current = 0;
		//apply an floating avg
		smoothCurrent = ( smoothCurrent*9 + current )/10;
		//mind maxima current
		if( smoothCurrentMax < smoothCurrent )
			smoothCurrentMax = smoothCurrent;
		//calc integral current consumption
		cap += current/200/3600;
		//measure temperature on ICE
		tmp = analogRead(6);
		rtemp = tmp*3120.0/4095/1000;
		rtemp = 3.3*1.2/rtemp-1.2;
		temperature = 79*rtemp-53;
		//measure handle controller
		tmp = 0;//analogRead(4);
		if(startDvs==1){
			tmp= 600;
		}

		/*if(tmp>500  )//&& tmp<1000)
			pwm1 = 1200*PCA0_MKS;//plug
		else
			pwm1 = 0;//plug*/

		if(tmp>500 ){//&& tmp<1000)
			pwm2 = 1500*PCA0_MKS;//starter
			if(chgCurrSmooth>2)
				pwm2 = 700*PCA0_MKS;
		}else{
			pwm2 = 700*PCA0_MKS;
		}
		if(tmp>500){// && tmp<1000)
			pwm3 = 1640*PCA0_MKS;//holostoi//zaslonka//bilo1050
			if(chgCurrSmooth>2)
				pwm3 = 1750*PCA0_MKS;
		}else{
			pwm3 = 1200*PCA0_MKS;//0//1320*/
		}

		if(temperature>40){
			pwm4 = (750)*PCA0_MKS;
		}else{
			pwm4 = 700*PCA0_MKS;//cooler
		}
		if(temperature>60){
			pwm4 = (750+(temperature-50)*50)*PCA0_MKS;//cooler
		}
		if(pwm4>2200*PCA0_MKS)
			pwm4=2200*PCA0_MKS;

		//voltage
		tmp = analogRead(4);
		if(bat != bat3_pr)
			flMem = 1;
		rtemp = 25.0/25.8*tmp*3120.0/4095.0*(3+22)/3.0/1000.0;
		bat = ( bat*9 + rtemp )/10;
		bat3_pr = bat;

		Wh = cap*bat;
		flRun = 1;
	}

	if (CountRun++ < 50)  
	WDTCN = 0xA5;	//
	SFRPAGE = SFRPAGE_SAVE;

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