#include "c8051f120.h"
#include "PCA0.h"

void PCA0_Init (void)
{
   char SFRPAGE_save = SFRPAGE;

   SFRPAGE = PCA0_PAGE;

   // Configure PCA time base; overflow interrupt disabled
   PCA0CN = 0x00;                      // Stop counter; clear all flags
   PCA0MD = 0x03;//0x04;                      // Use Timer 0 as time base

   PCA0CPM0 = 0x49;                    // Module 0 = Software Timer Mode,
                                       // Enable Module 0 Interrupt flag,
                                       // Enable ECOM bit
	PCA0CPM1 = 0x49;
	PCA0CPM2 = 0x49;
	PCA0CPM3 = 0x49;

   PCA0L = 0x00;                       // Reset PCA Counter Value to 0x0000
   PCA0H = 0x00;

   EIE1 |= 0x08;                       // Enable PCA interrupts

   CR = 1;                             // Start PCA

   SFRPAGE = TIMER01_PAGE;

   SFRPAGE = SFRPAGE_save;
}

