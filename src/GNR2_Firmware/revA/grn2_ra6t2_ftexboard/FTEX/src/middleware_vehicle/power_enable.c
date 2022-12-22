/**
  * @file    power_enable.c
  * @brief   This module handles power enable pin
  *
*/

#include "power_enable.h"
#include "vc_config.h"
#include "board_hardware.h"
#include "gnr_main.h"

extern osThreadId_t PowerOffSequence_handle;

/**
  Function used to initialise the GPIO pins used for the power enable module
*/
void PWREN_Init(PWREN_Handle_t * pHandle)
{    
    ASSERT(pHandle != NULL);

    struct GPIOConfig PinConfig;
    
    pHandle->bPowerCycle = false;
   
    PinConfig.PinDirection = INPUT;     //ReInit to ensure this pin has the wanted behavior
    PinConfig.PinPull      = NONE; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(PWREN_GPIO_PIN, PinConfig);
   
    PinConfig.PinDirection = OUTPUT;     //ReInit to ensure this pin has the wanted behavior
    PinConfig.PinPull      = NONE; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(CAN_STANDBY_N_GPIO_PIN, PinConfig);

}

/**
  Function used to check the state of the Power Enable signal
*/
bool PWREN_IsPowerEnabled(PWREN_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    if (pHandle->bUsePowerLock)
    {          
        pHandle->bIsPowerEnabled =  uCAL_GPIO_Read(PWREN_GPIO_PIN) ^ pHandle-> bIsInvertedLogic;
        
        return pHandle->bIsPowerEnabled;
    }
    else
    {
        return true;
    }       
}

/**
   Function used to monitor the current state of the Power Enable signal 
   and trigger the power off sequence if needed
*/
void PWREN_MonitorPowerEnable(PWREN_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    // If the power is off and it was on (falling edge), start the power off sequence
    if(!PWREN_IsPowerEnabled(pHandle) && pHandle->bPowerCycle)        
    {   
        osThreadFlagsSet(PowerOffSequence_handle,POWEROFFSEQUENCE_FLAG);
    }
    else if (PWREN_IsPowerEnabled(pHandle))
    {
        pHandle->bPowerCycle = true; // REgister that the power has been turned on
    }        
} 

/**
   Function used to turn off the power to the controller
*/
void PWREN_StopPower(PWREN_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);

    uCAL_GPIO_Reset(CAN_STANDBY_N_GPIO_PIN);
}