/**
  ******************************************************************************
  * @file    can_vehicle_interface.c
  * @author  Jabrane Chakroun, FTEX
  * @brief   High level module that transfer the can module needed parameters
  *          from the vehicle
  ******************************************************************************
*/

#include "can_vehicle_interface.h"
#include "vc_errors_management.h"
#include "ASSERT_FTEX.h"
#include "Utilities.h"
#include "wheel.h"
// disable warning about user_config_task modifying the pragma pack value
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpragma-pack"
#include "user_config_task.h"
#pragma clang diagnostic pop


bool CANScreenSetup = false;

// ==================== Public function prototypes ======================== //

/**
 *  @brief  Send long messages function
 *  @return None
 */
void CanVehiInterface_sendLongMsgs(uint8_t * message, uint8_t * dataToSend, uint16_t length)
{
    uint8_t BufferInit[MAX_DATA_IN_INIT];
    uint8_t * tmp_buffer;
    
    tmp_buffer = BufferInit;
    
    memcpy(tmp_buffer, dataToSend, length);
    
    for(int i = 0; i < MAX_DATA_IN_INIT; i++)
    {
        // Send data from byte 4
        message[i+BYTE_4_INDEX] = tmp_buffer[MAX_DATA_IN_INIT-(1+i)]; 
    }
}
// ==================== Public function prototypes ======================== //

/**
 *  Get vehicle DC power
 */
uint16_t CanVehiInterface_GetVehicleDCPower(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);    
    return PWRT_GetDCPower(pHandle->pPowertrain);
}

/**
 *  Get vehicle torque
 */
uint16_t CanVehiInterface_GetVehicleTorque(VCI_Handle_t * pHandle)
{
    float TotalVehicleTorque = 0;
    ASSERT(pHandle!= NULL);
    
    // Add up motor torque reference from both motors
    TotalVehicleTorque = (float)PWRT_GetTotalMotorsTorque(pHandle->pPowertrain);
    
    TotalVehicleTorque = TotalVehicleTorque * MDI_GetMotorGearRatio(pHandle->pPowertrain->pMDI);
    
    return (uint16_t)round(TotalVehicleTorque/100);
}

/**
 *  Get vehicle power
 */
uint16_t CanVehiInterface_GetVehiclePower(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint16_t hmsgToSend;
    hmsgToSend = PWRT_GetTotalMotorsPower(pHandle->pPowertrain);
    return hmsgToSend;
}

/**
 *  Get vehicle state of charge
 */
uint8_t CanVehiInterface_GetVehicleSOC (VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint8_t bSoC;
    bSoC = (uint8_t)BatMonitor_GetSOC (pHandle->pPowertrain->pBatMonitorHandle);
    return bSoC;
}

/**
 *  Get vehicle PAS
 */
uint8_t CanVehiInterface_GetVehiclePAS (VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint8_t bPas;
    bPas = PedalAssist_GetAssistLevel(pHandle->pPowertrain->pPAS);
    return bPas;
}

/**
 *  Set vehicle PAS
 */
void CanVehiInterface_SetVehiclePAS (VCI_Handle_t * pHandle, uint8_t Set_PAS)
{
    ASSERT(pHandle!= NULL);
    PedalAssist_SetAssistLevel(pHandle->pPowertrain->pPAS, Set_PAS);
}

/**
 *  Get vehicle maximum PAS
 */
uint8_t CanVehiInterface_GetVehicleMaxPAS (void)
{
    uint8_t bMaxPas;
    bMaxPas = UserConfigTask_GetNumberPasLevels();
    return bMaxPas;
}

/**
 *  Get max DC power
 */
uint16_t CanVehiInterface_GetMaxDCPWR (VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL); 
    return PWRT_GetMaxDCPower(pHandle->pPowertrain);
}

/**
 *  Getbit map of vehicle current faults
 */
uint32_t CanVehiInterface_GetVehicleCurrentFaults (VCI_Handle_t * pHandle)
{   
    ASSERT(pHandle!= NULL);    
    return VC_Errors_GetErrorBitMap();
}

/**
 *  Get vehicle firmware version
 */
uint16_t CanVehiInterface_GetVehicleFwVersion(void)
{
    uint16_t hFWVersion;
    hFWVersion = FW_VERSION;
    return hFWVersion;
}

/**
 *  Get vehicle hardware version
 */
uint16_t CanVehiInterface_GetVehicleHwVersion(void)
{
    uint16_t hHWVersion;
    hHWVersion = HW_VERSION;
    return hHWVersion;
}

/**
 *  Get vehicle serial number
 */
uint8_t CanVehiInterface_GetVehicleSerialNumber(void)
{
    uint8_t msgToSend;
    CanVehiInterface_sendLongMsgs(&msgToSend, (uint8_t *)SERIAL_NUMBER, strlen(SERIAL_NUMBER)/2);
    return msgToSend;
}

/**
 *  Get the wheel diamater used to calculate speed
 */
uint8_t CanVehiInterface_GetWheelDiameter(void)
{
   return Wheel_GetWheelDiameter();
}

/**
 *  Update wheel diameter used to calculate speed
 */
void CanVehiInterface_UpdateWheelDiameter(uint8_t aDiameterInInches)
{
    if (10 < aDiameterInInches && aDiameterInInches < 40) // Temporary safety net until CAN screen integration is complete
    {
       Wheel_ExternalSetWheelDiameter(aDiameterInInches);
    }        
}

/**
 *  Get vehicle Speed
 */
uint16_t CanVehiInterface_GetVehicleSpeed(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pPowertrain != NULL);
    ASSERT(pHandle->pPowertrain->pPAS != NULL);
    ASSERT(pHandle->pPowertrain->pPAS->pWSS != NULL);
    
    uint16_t hmsgToSend;

    hmsgToSend = Wheel_GetVehicleSpeedFromWSS(pHandle->pPowertrain->pPAS->pWSS);

    return hmsgToSend;
}


/**
 *  Get the current state of the front light 
 */
uint8_t CanVehiInterface_GetFrontLightState(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    return (uint8_t) Light_GetState(pHandle->pPowertrain->pHeadLight);
}

/**
 *  Change the current state of the front light 
 */
void CanVehiInterface_ChangeFrontLightState(VCI_Handle_t * pHandle, uint8_t aState)
{
    ASSERT(pHandle!= NULL);
    

    if(aState)
    {        
        Light_Enable(pHandle->pPowertrain->pHeadLight);
        Light_ClearInternalUpdateFlag(pHandle->pPowertrain->pHeadLight);
    }
    else
    {
        Light_Disable(pHandle->pPowertrain->pHeadLight);
        Light_ClearInternalUpdateFlag(pHandle->pPowertrain->pHeadLight);
    }      
}

/**
 *  Get the current state of the rear light 
 */
uint8_t CanVehiInterface_GetRearLightState(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    return (uint8_t) Light_GetState(pHandle->pPowertrain->pTailLight);
}

/**
 *  Change the current state of the rear light 
 */
void CanVehiInterface_ChangeRearLightState(VCI_Handle_t * pHandle, uint8_t aState)
{
    ASSERT(pHandle!= NULL);
          
    if(aState)
    {        
        Light_Enable(pHandle->pPowertrain->pTailLight);
        Light_ClearInternalUpdateFlag(pHandle->pPowertrain->pTailLight);
    }
    else
    {
        Light_Disable(pHandle->pPowertrain->pTailLight);
        Light_ClearInternalUpdateFlag(pHandle->pPowertrain->pTailLight);
    }
}

/**
  *  Getting the controller NTC temperature value
  */
int16_t CanVehiInterface_GetControllerTemp(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MDI_GetControllerTemp(pHandle->pPowertrain->pMDI);
}

/**
  *  Getting the motor NTC temperature value
  */
int16_t CanVehiInterface_GetMotorTemp(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MDI_GetMotorTemp(pHandle->pPowertrain->pMDI);
}

/**
 *  Pass by reference all PAS values of the minimum torque percentage
 */
void CanVehiInterface_GetPasLevelMinTorque(VCI_Handle_t * pHandle, uint8_t * pasLevelMinTorque)
{
   ASSERT(pHandle != NULL); 
   for(uint8_t n = PAS_LEVEL_0;n <= PAS_LEVEL_9;n++)
   {
        pasLevelMinTorque[n] = pHandle->pPowertrain->pPAS->sParameters.PASMinTorqRatiosInPercentage[n];
   }
}

/**
  Setup the vehicle to use a CAN screen
 */
void CanVehiInterface_SetupCANScreen(VCI_Handle_t * pHandle)
{
   ASSERT(pHandle != NULL);
   ASSERT(UART0Handle.UARTProtocol == UART_DISABLE);  // We can only call this when no UART screen is present
    
   Throttle_SetupExternal(pHandle->pPowertrain->pThrottle,65535,0); 
   CANScreenSetup = true; 
}

/**
  Check if we are setup for a CAN screen
 */
bool CanVehiInterface_CheckCANScreenSetup(void)
{
    return CANScreenSetup;
}

/**
  Provide the vc layer with an updated Throttle value
 */
void CanVehiInterface_UpdateExternalThrottle(VCI_Handle_t * pHandle, uint16_t aNewThrottleVal)
{
   if(CANScreenSetup) // Check if we are expecting a CAN screen
   {
       Throttle_UpdateExternal(pHandle->pPowertrain->pThrottle,aNewThrottleVal);
   }       
   else
   {
       ASSERT(false);
   }
}

/**
  Enables the can layer to engage cruise control
 */
void CanVehiInterface_EngageCruiseControl(VCI_Handle_t * pHandle)
{
     uint8_t currentSpeed = (uint8_t)Wheel_GetSpeedFromWheelRpm(WheelSpdSensor_GetSpeedRPM(pHandle->pPowertrain->pPAS->pWSS));
                
     //check if the current speed is inside of the max speed limit
     //if not used MaxThrottleSpeedKMH as cruise control speed.
     if (currentSpeed > pHandle->pPowertrain->pThrottle->hParameters.MaxThrottleSpeedKMH)
     {
         currentSpeed = (uint8_t)pHandle->pPowertrain->pThrottle->hParameters.MaxThrottleSpeedKMH;
     }
                
     PWRT_EngageCruiseControl(pHandle->pPowertrain,currentSpeed);
}

/**
  Enables the can layer to disengage cruise control
 */
void CanVehiInterface_DisengageCruiseControl(VCI_Handle_t * pHandle)
{
    PWRT_DisengageCruiseControl(pHandle->pPowertrain);
}

/**
  Gives the can layer acces to the current state of the cruise control
 */
bool CanVehiInterface_GetCruiseControlState(VCI_Handle_t * pHandle)
{
    return PWRT_GetCruiseControlState(pHandle->pPowertrain);
}    

