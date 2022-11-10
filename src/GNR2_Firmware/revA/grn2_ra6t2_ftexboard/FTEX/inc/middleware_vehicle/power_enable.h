/**
  * @file    power_enable.h
  * @brief   This module handles power enable pin
  *
*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWER_ENABLE_H
#define __POWER_ENABLE_H

#include "stdbool.h"
#include "uCAL_GPIO.h"
#include "ASSERT_FTEX.h"

#define POWEROFFSEQUENCE_FLAG  0x63 //TODO move to a dedicated flag file

typedef struct
{          
	bool bUsePowerLock;
	bool bIsPowerEnabled;
	bool bIsInvertedLogic;
	
} PWREN_Handle_t;

/**
  @brief Function used to initialise the GPIO pins used for the power enable module
  @param pHandle : Pointer on Handle structure of PWREN module
  @return void
*/
void PWREN_Init(PWREN_Handle_t * pHandle);

/**
  @brief Function used to check the state of the Power Enable signal
  @param pHandle : Pointer on Handle structure of PWREN module
  @return bool
*/
bool PWREN_IsPowerEnabled(PWREN_Handle_t * pHandle);

/**
  @brief Function used to monitor the current state of the Power Enable signal 
         and trigger the power off sequence if needed
	
  @param pHandle : Pointer on Handle structure of PWREN module
  @return void
*/
void PWREN_MonitorPowerEnable(PWREN_Handle_t * pHandle);

/**
  @brief Function used to turn off the power to the controller
	
  @param pHandle : Pointer on Handle structure of PWREN module
  @return void
*/
void PWREN_StopPower(PWREN_Handle_t * pHandle);

#endif /*__POWER_ENABLE_H*/

