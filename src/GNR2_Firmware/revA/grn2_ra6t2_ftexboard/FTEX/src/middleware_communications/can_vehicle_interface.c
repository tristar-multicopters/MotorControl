/**
  ******************************************************************************
  * @file    can_vehicle_interface.c
  * @author  Jabrane Chakroun, FTEX
  * @brief   High level module that transfer the can module needed parameters
  *          from the vehicle
  ******************************************************************************
*/

#include "can_vehicle_interface.h"
#include "ASSERT_FTEX.h"
#include "Utilities.h"
// disable warning about user_config_task modifying the pragma pack value
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpragma-pack"
#include "user_config_task.h"
#pragma clang diagnostic pop

// ==================== Public function prototypes ======================== //

/**
  @brief  Send long messages function
  @return None
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
 *  Get vehicle power
 */
uint16_t CanVehiInterface_GetVehiclePower(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint16_t hmsgToSend;
    
    // Get Current from motor drive layer
    uint16_t current = PWRT_ConvertDigitalCurrentToAMPS(pHandle->pPowertrain, (uint16_t) pHandle->pPowertrain->pMDI->pMCI->pFOCVars->Iqdref.q);   
    
    // Get Voltage from motor drive layer
    uint16_t voltage = MCInterface_GetBusVoltageInVoltx100(pHandle->pPowertrain->pMDI->pMCI)/100;
    
    // Calculate the power
    uint16_t power = current * voltage;
    
    // Load data buffer
    hmsgToSend = power;   
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
 *  Get vehicle max power
 */
uint16_t CanVehiInterface_GetVehicleMaxPWR (VCI_Handle_t * pHandle)
{
    uint16_t bMaxPWR;
    bMaxPWR = (MCInterface_GetBusVoltageInVoltx100(pHandle->pPowertrain->pMDI->pMCI)/100) * PWRT_GetOngoingMaxCurrent(pHandle->pPowertrain);
    return bMaxPWR;
}

/**
 *  Get vehicle current faults
 */
uint16_t CanVehiInterface_GetVehicleCurrentFaults (VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint16_t hCurrFault;
    hCurrFault = MDI_GetCurrentFaults(pHandle->pPowertrain->pMDI,M1)|
                 MDI_GetCurrentFaults(pHandle->pPowertrain->pMDI,M2);
    
    uint16_t stateToSend = MC_NO_ERROR;
      
    if(hCurrFault != MC_NO_ERROR)
    {
        // Motor over temperature fault
        if(hCurrFault & MC_OVER_TEMP)
        {
            stateToSend |= MOTOR_OVER_T_FAULT;
        }
        // Motor start up fault
        if(hCurrFault & MC_START_UP)
        {
            stateToSend |= MOTOR_START_U_FAULT;
        }
        // Motor over current fault
        if(hCurrFault & MC_BREAK_IN)
        {
            stateToSend |= MOTOR_OVER_C_FAULT;
        }
        // Motor speed feedback fault
        if(hCurrFault & MC_SPEED_FDBK)
        {
            stateToSend |= HALL_SENSOR_FAULT;
        }
        // Motor over voltage fault
        if(hCurrFault & MC_OVER_VOLT)
        {
            stateToSend |= CONTROL_OVER_V;
        }
        // Motor under voltage
        if(hCurrFault & MC_UNDER_VOLT)
        {
            stateToSend |= CONTROL_UNDER_V;
        }
    }
    return stateToSend;
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
    Update wheel diameter used to calculate speed
*/
void CanVehiInterface_UpdateWheelDiameter(uint8_t aDiameterInInches)
{
    if (10 < aDiameterInInches && aDiameterInInches < 40) // Temporary safety net until CAN screen integration is complete
    {
        Wheel_SetWheelDiameter(aDiameterInInches);
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
  Get the current state of the front light 
*/
uint8_t CanVehiInterface_GetFrontLightState(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    return (uint8_t) Light_GetState(pHandle->pPowertrain->pHeadLight);
}

/**
  Change the current state of the front light 
*/
void CanVehiInterface_ChangeFrontLightState(VCI_Handle_t * pHandle, uint8_t aState)
{
    ASSERT(pHandle!= NULL);
    if(aState == true)
    {        
        Light_Enable(pHandle->pPowertrain->pHeadLight);
    }
    else
    {
        Light_Disable(pHandle->pPowertrain->pHeadLight);
    }        
}

/**
  Get the current state of the rear light 
*/
uint8_t CanVehiInterface_GetRearLightState(VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    return (uint8_t) Light_GetState(pHandle->pPowertrain->pTailLight);
}

/**
  Change the current state of the rear light 
*/
void CanVehiInterface_ChangeRearLightState(VCI_Handle_t * pHandle, uint8_t aState)
{
    ASSERT(pHandle!= NULL);
    if(aState)
    {        
        Light_Enable(pHandle->pPowertrain->pTailLight);
    }
    else
    {
        Light_Disable(pHandle->pPowertrain->pTailLight);
    }
    
}
