#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdio.h>
#include <NUC029FAE.h>
#include <gpio.h>
#include <stdbool.h>
#include "flasher.h"

void SYS_Init(void)
{
  /* Unlock protected registers */
  SYS_UnlockReg();

  /* Enable internal 22.1184MHz */
  CLK->PWRCON = CLK_PWRCON_IRC22M_EN_Msk;

  while (!CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk))
    ;

  /* Start UART Clock */
	#if DEBUG_MODE
		CLK->APBCLK |= CLK_APBCLK_UART_EN_Msk;
  #endif
  /* Update System Core Clock */
  /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
  SystemCoreClockUpdate();

  /* Set P1 multi-function pins for UART RXD, TXD */
  SYS->P1_MFP = SYS_MFP_P12_RXD | SYS_MFP_P13_TXD;

  /* Lock protected registers */
  SYS_LockReg();

  /* Initializes UART Communication */
	#if DEBUG_MODE
		UART_Open(UART, 115200);
		printf("init\n");
  #endif
  /* Initializes Ticker Library*/
  startTicker(TIMER1);
}

#endif
