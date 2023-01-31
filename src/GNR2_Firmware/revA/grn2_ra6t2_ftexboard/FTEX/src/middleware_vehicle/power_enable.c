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
   
    PinConfig.PinDirection = INPUT;     //ReInit to ensure this pin has the wanted behavior
    PinConfig.PinPull      = NONE; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(PWR_ENABLE_GPIO_PIN, PinConfig);
   
    PinConfig.PinDirection = OUTPUT;     //ReInit to ensure this pin has the wanted behavior
    PinConfig.PinPull      = NONE; 
    PinConfig.PinOutput    = PUSH_PULL; 
    
    uCAL_GPIO_ReInit(CAN_STANDBY_N_GPIO_PIN, PinConfig);
    
    if(PWREN_IsPowerEnabled(pHandle)) 
    { 
        pHandle->bInitialPowerLockState = true;   
    }
    else
    {
        pHandle->bInitialPowerLockState = false;   
    }
    
    //PWREN struct was initialized.
    pHandle->bInitalized = true;

}

/**
  Function used to check the state of the Power Enable signal
*/
bool PWREN_IsPowerEnabled(PWREN_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    if (pHandle->bUsePowerLock)
    {          
        pHandle->bIsPowerEnabled = uCAL_GPIO_Read(PWR_ENABLE_GPIO_PIN) ^ pHandle-> bIsInvertedLogic;
        
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
    
    // test if display resquested the turn off.
    if((PWREN_IsPowerEnabled(pHandle) == false) && (pHandle->bInitialPowerLockState == true))        
    {   
        osThreadFlagsSet(PowerOffSequence_handle,POWEROFFSEQUENCE_FLAG);
    }
    
    //test if was false turn on command received by CAN interface 
    if((PWREN_IsPowerEnabled(pHandle) == false) &&(pHandle->bWakeUpCommandChecked == true) && (pHandle->bWakeUpSDOCommand == false))
    {
        
        osThreadFlagsSet(PowerOffSequence_handle,POWEROFFSEQUENCE_FLAG);
        
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

/**
  @brief Function to check if the firmware update command was received.
  @param PWREN_Handle_t * pHandle pointer to the struct responsible
         to give access to the flags used to know if the device was turned on
         by firmware command received by CAN or was a false wake up.
  @param uint8_t UpdateCommand used to pass the value read from the CAN dictionary 
         address responsible to hold the firmware update command.
  @return void
*/
void PWREN_CheckFirmwareUpdateCommand(PWREN_Handle_t * pHandle, uint16_t UpdateCommand)
{
    
    ASSERT(pHandle != NULL);
    
    //variable used to check the timeout to receive a firmware update command.
    static uint16_t firmwareUpdateTimeout = 0;   

    //Start to check if the firmware update command was received.\
    //This command afect the way the PWREM works(power on/off management).
    //If the PWREN was initialized the firmware update check request can be made.
    if((VCInterfaceHandle.pPowertrain->pPWREN->bInitalized == true) && (VCInterfaceHandle.pPowertrain->pPWREN->bWakeUpSDOCommand == false))
    {     
        //Check if the the device received a wake up command by SDO to start the bootloader mode(firmware update).
        if(UpdateCommand == FIRMWARE_UPDATE_REQUEST)
        {
            //Set the flag to say if the command was received or a timeout happened.
            //This decision will be made by the PWREN.
            // bWakeUpSDOCommand == true and bWakeUpSDOCommandChecked == true
            //means command was received.
            VCInterfaceHandle.pPowertrain->pPWREN->bWakeUpCommandChecked = true;
                    
            //Set the the correspondent flag to indicate that a firmware update request arrived.
            VCInterfaceHandle.pPowertrain->pPWREN->bWakeUpSDOCommand = true; 
        }
                
        //Check if the device was woken up by the a CAN event.
        //if was start to count the timeout.                
        if(VCInterfaceHandle.pPowertrain->pPWREN->bInitialPowerLockState == false)
        {
            //
            firmwareUpdateTimeout++;
        }
                
        //if in 5 seconds the device don't receive a firmware update command, he will set
        //the flag above.
        if((firmwareUpdateTimeout > FIRMWAREUPDATE_TIMEOUT) && (VCInterfaceHandle.pPowertrain->pPWREN->bWakeUpCommandChecked == false))
        {
            //Set the flag to say if the command was received or a timeout happened.
            //This decision will be made by the PWREN.
            // bWakeUpSDOCommand == false and bWakeUpSDOCommandChecked == true
            //means command was not received.
            VCInterfaceHandle.pPowertrain->pPWREN->bWakeUpCommandChecked = true;
        } 
    }
}