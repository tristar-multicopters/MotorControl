/**
  ******************************************************************************
  * @file    can_iot_comm.c
  * @author  Jabrane Chakroun, FTEX
  * @brief   High level module that transfer the IOT module needed parameters
  *          via CanOpen
  ******************************************************************************
*/

#include "can_iot_comm.h"
#include "ASSERT_FTEX.h"
// ==================== Public function prototypes ======================== //

/**
  @brief  IOT CanOpen  Send long messages function
  @return None
*/
void CanIot_sendLongMsgs(uint8_t * message, uint8_t * dataToSend, uint16_t length)
{
    uint8_t * tmp_buffer = 0;
    memcpy(tmp_buffer, dataToSend, length);
    
    for(int i = 0; i < MAX_DATA_IN_INIT; i++)
    {
        // Send data from byte 4
        message[i+BYTE_4_INDEX] = tmp_buffer[MAX_DATA_IN_INIT-(1+i)]; 
    }
}
// ==================== Public function prototypes ======================== //

/**
 *  IOT CanOpen Get Speed
 */
uint8_t CanIot_GetSpeed (VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint8_t hmsgToSend;
    // Get the RPM wheel from the wheel speed sensor module
    int16_t RpmSpeed = (int16_t)WheelSpdSensor_GetSpeedRPM(pHandle->pPowertrain->pPAS->pWSS);
    // Convert the measurement in km/h;
    RpmSpeed = (uint8_t)((float)RpmSpeed*RPM_TO_KM); 

    // Load data buffer
    hmsgToSend = (uint8_t)RpmSpeed;

    return hmsgToSend;
}

/**
 *  IOT CanOpen Get Power
 */
uint16_t CanIot_GetPower (VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint16_t hmsgToSend;
    // Get Current from motor drive layer
    uint8_t Current = abs(pHandle->pPowertrain->pMDI->pMCI->pFOCVars->Iqdref.q);
    // Get Volatage from motor drive layer
    uint16_t Voltage = MCInterface_GetBusVoltageInVoltx100(pHandle->pPowertrain->pMDI->pMCI);
    // Calculate the power
    uint16_t Power = ((uint16_t)Current * Voltage)/100;
    
     // Load data buffer
    hmsgToSend = Power;   
    return hmsgToSend;

}

/**
 *  IOT CanOpen Get State of Charge
 */
uint8_t CanIot_GetSOC (VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint8_t bSoC;
    bSoC = (uint8_t)BatMonitor_GetSOC (pHandle->pPowertrain->pBatMonitorHandle);
    return bSoC;
}

/**
 *  IOT CanOpen Get PAS
 */
uint8_t CanIot_GetPAS (VCI_Handle_t * pHandle)
{
    ASSERT(pHandle!= NULL);
    uint8_t bPas;
    bPas = PedalAssist_GetAssistLevel(pHandle->pPowertrain->pPAS);
    return bPas;
}

/**
 *  IOT CanOpen Set PAS
 */
void CanIot_SetPAS (VCI_Handle_t * pHandle, uint8_t Set_PAS)
{
    ASSERT(pHandle!= NULL);
    PedalAssist_SetAssistLevel(pHandle->pPowertrain->pPAS, Set_PAS);
}

/**
 *  IOT CanOpen Get Maximum PAS
 */
uint8_t CanIot_GetMaxPAS (void)
{
    uint8_t bMaxPas;
    bMaxPas = PAS_MAX_LEVEL;
    return bMaxPas;
}

/**
 *  IOT CanOpen Get MAX Power
 */
uint16_t CanIot_GetMaxPWR (void)
{
    uint16_t bMaxPWR;
    bMaxPWR = (uint8_t)NOMINAL_TORQUE;
    return bMaxPWR;
}

/**
 *  IOT CanOpen Get Current Faults
 */
uint16_t CanIot_GetCurrentFaults (VCI_Handle_t * pHandle)
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
 *  Initializes battery monitor module
 */
uint16_t CanIot_GetFwVersion(void)
{
    uint16_t hFWVersion;
    hFWVersion = FW_VERSION;
    return hFWVersion;
}

/**
 *  IOT CanOpen Get Hardware version
 */
uint16_t CanIot_GetHwVersion(void)
{
    uint16_t hHWVersion;
    hHWVersion = HW_VERSION;
    return hHWVersion;
}

/**
 *  IOT CanOpen Get Serial Number
 */
uint8_t CanIot_GetSerialNumber(void)
{
    uint8_t msgToSend;
    CanIot_sendLongMsgs(&msgToSend, (uint8_t *)SERIAL_NUMBER, strlen(SERIAL_NUMBER)/2);
    return msgToSend;
}
