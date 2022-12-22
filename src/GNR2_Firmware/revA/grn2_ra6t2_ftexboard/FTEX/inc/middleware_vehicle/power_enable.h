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

#define POWEROFFSEQUENCE_FLAG  0x13  // OS flag used to block the power enable task
                                     // TODO move to a dedicated flag file

typedef struct
{          
    bool bUsePowerLock;     /* Tells use if this bike uses a power lock */
    bool bIsPowerEnabled;   /* Tells us if the power is currently enable (we can push power) */
    bool bIsInvertedLogic;  /* Tells use if the pin used to check the state of the power is inverted */
    bool bPowerCycle;       /* Tells us if the power has already been turned on so 
                                     we can have a fallign edge to power off */ 
    
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

