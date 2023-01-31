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

// value used to check if a firmware update request was made by SDO service.
#define FIRMWARE_UPDATE_REQUEST  0x2BE5

//timeout to wait for the firmware update command.
//Timeout == (FIRMWAREUPDATE_TIMEOUT*25ms) seconds. 
//Timeout == 200*25ms = 5 seconds.
#define FIRMWAREUPDATE_TIMEOUT  200 

typedef struct
{          
    bool bInitalized;       /* Tells use if the PWREN struct was initialized or not.*/
    bool bUsePowerLock;     /* Tells use if this bike uses a power lock */
    bool bIsPowerEnabled;   /* Tells us if the power is currently enable (we can push power) */
    bool bIsInvertedLogic;  /* Tells use if the pin used to check the state of the power is inverted */
    bool bInitialPowerLockState;       /* Tells us who has woke up the GNR*/ 
    bool bWakeUpSDOCommand; /* Tells us if the device was woke up by a SDO wake up command.*/
    bool bWakeUpCommandChecked;
   
    
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

/**
  @brief Function to check if the firmware update command was received.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param uint8_t UpdateCommand used to pass the value read from the CAN dictionary 
         address responsible to hold the firmware update command.
  @return void
*/
void PWREN_CheckFirmwareUpdateCommand(PWREN_Handle_t * pHandle, uint16_t UpdateCommand);

#endif /*__POWER_ENABLE_H*/

