/**
 *******************************************************************************
 * @file  system_RIN32M4.c
 * @brief sample program for R-IN32M4
 * 
 * @note 
 * Copyright (C) 2015 Renesas Electronics Corporation 
 * Copyright (C) 2015 Renesas System Design Co., Ltd. 
 * 
 * @par
 *  This is a sample program. 
 *  Renesas Electronics assumes no responsibility for any losses incurred. 
 * 
 * modified by ARM
 *******************************************************************************
 */

/*============================================================================*/
/* I N C L U D E                                                              */
/*============================================================================*/
#include "RIN32M4.h"
#include "system_RIN32M4.h"

/*============================================================================*/
/* D E F I N E                                                                */
/*============================================================================*/
/* CC-Link IE WDT */
#define CCIWDT							(*((volatile uint16_t *) (0x40100962)))

/*============================================================================*/
/* V A R I A B L E                                                            */
/*============================================================================*/
uint32_t SystemCoreClock = RIN32M4_SYSCLK ;

/*============================================================================*/
/* P R O G R A M                                                              */
/*============================================================================*/
/**
 ******************************************************************************
  @brief  System initialize
  @param  none
  @retval none
 ******************************************************************************
*/
#ifndef __ICCARM__									// if not use IAR compiler
__attribute__ ((section("handlers_rom"))) 
#endif
void SystemInit(void)
{

	// --- Setup Cortex-M4 core ---
	SCB->SHCSR = 0x00070000;						// Bus,Usage,Mem Fault Handler Enable
	CCIWDT = 0x0000;								// Disable watch dog timer in CC-Link IE module

	// --- Setup memory controller ---
	// ToDo: Please modify wait setting according to user board. Default setting is slowest.
	RIN_MEMC->SMC0 = 0x0000ffff;					// Access wait setting for CS0
//	RIN_MEMC->PRC  = 0x20010001;					// Page ROM control for CS0
	RIN_MEMC->SMC1 = 0x0000ffff;					// Access wait setting for CS1
	RIN_MEMC->SMC2 = 0x0000ffff;					// Access wait setting for CS2
	RIN_MEMC->SMC3 = 0x0000ffff;					// Access wait setting for CS3

#if (__FPU_USED == 1)
  /* enable FPU if available and used */
  SCB->CPACR |= ((3UL << 10*2) |             /* set CP10 Full Access               */
                 (3UL << 11*2)  );           /* set CP11 Full Access               */
#endif
}

/**
 ******************************************************************************
  @brief  System core clock update
  @param  none
  @retval none
 ******************************************************************************
*/
#ifndef __ICCARM__									// if not use IAR compiler
__attribute__ ((section("handlers_rom"))) 
#endif
void SystemCoreClockUpdate(void)
{

}

/**
 ******************************************************************************
  @brief  get driver version
  @param  [in] mode -- 0 : version , 1 : version + compile date
  @retval none
 ******************************************************************************
*/
char *rin_package_get_version(uint8_t mode)
{
	if(mode)
	{
		return RIN_PACKAGE_VERSION" (build:"__DATE__" "__TIME__")";
	}
	else
	{
		return RIN_PACKAGE_VERSION;
	}
}

/**
 ******************************************************************************
  @brief  get driver version
  @param  [in] mode -- 0 : version , 1 : version + compile date
  @retval none
 ******************************************************************************
*/
char *rin_driver_get_version(uint8_t mode)
{
	if(mode)
	{
		return RIN_DRIVER_VERSION" (build:"__DATE__" "__TIME__")";
	}
	else
	{
		return RIN_DRIVER_VERSION;
	}
}
