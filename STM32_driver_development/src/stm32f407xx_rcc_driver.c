/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 30-Jul-2019
 *      Author: Sutej Kulkarni
 */

#include "stdint.h"

uint32_t AHB1_PRESCALARS[] = {2,4,8,16,64,128,256,512};
uint32_t APB1_PRESCALARS[] = {2,4,8,16};

#include "stm32f407xx_rcc_driver.h"

uint32_t RCC_GetPCLK1Value(void)
{
   uint32_t pclk1,SysClk;

   uint8_t clksrc,ahb1ps,apb1ps,temp;

   clksrc = ((RCC->CFGR >> 2) & (0X3));

   if(clksrc == 0)
   {
	   SysClk = 160000000;
   }
   else if(clksrc == 1)
   {
	   SysClk = 8000000;
   }
   else if(clksrc == 2 )
   {
	   SysClk = RCC_GetPLLOPClock();
   }

   //FOR AHB PRESCALAR
   temp = ((RCC->CFGR >> 4) & 0xF);

   if (temp < 0X8)
   {
	   ahb1ps = 1;
   }
   else
   {
       ahb1ps = AHB1_PRESCALARS[temp-8];
   }

   //FOR APB PRESCALAR

   temp = 0;
   temp = ((RCC->CFGR >> 10) & 0X7);

   if (temp < 0x4)
   {
	   apb1ps = 1;
   }
   else
   {
	   apb1ps = APB1_PRESCALARS[temp-4];
   }

   pclk1 = (SysClk/ahb1ps)/apb1ps;

   return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
   uint32_t pclk1,SysClk;

   uint8_t clksrc,ahb1ps,apb1ps,temp;

   clksrc = ((RCC->CFGR >> 2) & (0X3));

   if(clksrc == 0)
   {
	   SysClk = 160000000;
   }
   else if(clksrc == 1)
   {
	   SysClk = 8000000;
   }
   else if(clksrc == 2 )
   {
	   SysClk = RCC_GetPLLOPClock();
   }

   //FOR AHB PRESCALAR
   temp = ((RCC->CFGR >> 4) & 0xF);

   if (temp < 0X8)
   {
	   ahb1ps = 1;
   }
   else
   {
       ahb1ps = AHB1_PRESCALARS[temp-8];
   }

   //FOR APB PRESCALAR

   temp = 0;
   temp = ((RCC->CFGR >> 10) & 0X7);

   if (temp < 0x4)
   {
	   apb1ps = 1;
   }
   else
   {
	   apb1ps = APB1_PRESCALARS[temp-4];
   }

   pclk1 = (SysClk/ahb1ps)/apb1ps;

   return pclk1;
}


