/**
*  can_logger.c
*  Middleware layer for managing CAN logging messages
*/
#include "can_logger.h"
#if ENABLE_CAN_LOGGER
/* Function for sending the vehicle status */

void CANLOG_getStatus(VCI_Handle_t * pHandle, uint8_t bMotorSelection)
{
    can_frame_t msgToSend;
    uint32_t status = 0;
    // Prepare Header of CAN message 
    msgToSend.id_mode = CAN_ID_MODE_STANDARD;
    msgToSend.type    = CAN_FRAME_TYPE_DATA;
    msgToSend.data_length_code = 3;
    // Get status, fault now and occured faults
	if(bMotorSelection == M_NONE)
	{
		msgToSend.id = CAN_ID_STATUS_VC;
		status = ((uint32_t)VCSTM_GetState(pHandle->pStateMachine) << 16) |
				  (uint32_t)VCSTM_GetFaultState(pHandle->pStateMachine);
	}
	
	else if(bMotorSelection == M1)
	{
		msgToSend.id = CAN_ID_STATUS_M1;
		status = MDI_GetSTMState(pHandle->pPowertrain->pMDI, bMotorSelection)                |
						 (uint32_t)MDI_GetCurrentFaults(pHandle->pPowertrain->pMDI, bMotorSelection)  <<  8 |
						 (uint32_t)MDI_GetOccurredFaults(pHandle->pPowertrain->pMDI, bMotorSelection) << 16;
	}
	else
	{
		msgToSend.id = CAN_ID_STATUS_M2;
		status = MDI_GetSTMState(pHandle->pPowertrain->pMDI, bMotorSelection)                |
						 (uint32_t)MDI_GetCurrentFaults(pHandle->pPowertrain->pMDI, bMotorSelection)  <<  8 |
						 (uint32_t)MDI_GetOccurredFaults(pHandle->pPowertrain->pMDI, bMotorSelection) << 16;
	}
	// Load data buffer
	msgToSend.data[0] = status & 0xFF;     // Fault occurred
	msgToSend.data[1] = (status >> 8)  & 0xFF; // Fault now
	msgToSend.data[2] = (status >> 16) & 0xFF; // State
    
    //Send CAN message
    uCAL_CAN_SendMsg(msgToSend);
}

/* Function for sending the bus voltage value*/
void CANLOG_getVbus(VCI_Handle_t * pHandle)
{
    can_frame_t msgToSend;
    // TO DO: Add a function in the MDI module for getting the Bus voltage
    // Prepare Header of CAN message 
    uint8_t bVoltage = 0x64;
    msgToSend.id = CAN_ID_VBUS;
    msgToSend.id_mode = CAN_ID_MODE_STANDARD;
    msgToSend.type    = CAN_FRAME_TYPE_DATA;
    msgToSend.data_length_code = 1;
    // Load data buffer
    msgToSend.data[0] = bVoltage;
    
    //Send CAN message
    uCAL_CAN_SendMsg(msgToSend);
}

/* Function for sending the reference and measured currents */
void CANLOG_getCurrent(VCI_Handle_t * pHandle, uint8_t bMotorSelection)
{
    can_frame_t msgToSend;
    // Get current values
    int16_t hCurrentRefq = MDI_GetIqdref(pHandle->pPowertrain->pMDI, bMotorSelection).q;
    int16_t hCurrentRefd = MDI_GetIqdref(pHandle->pPowertrain->pMDI, bMotorSelection).d;
    int16_t hCurrentMeasq = MDI_GetIqd(pHandle->pPowertrain->pMDI, bMotorSelection).q;
    int16_t hCurrentMeasd = MDI_GetIqd(pHandle->pPowertrain->pMDI, bMotorSelection).d;
    // Prepare Header of CAN message 
    if(bMotorSelection == M1)
        msgToSend.id = CAN_ID_CURRENT_M1;
	else
		msgToSend.id = CAN_ID_CURRENT_M2;
    msgToSend.id_mode = CAN_ID_MODE_STANDARD;
    msgToSend.type    = CAN_FRAME_TYPE_DATA;
    msgToSend.data_length_code = 8;
    
    // Load data buffer
    msgToSend.data[0] = hCurrentRefq & 0xFF;
    msgToSend.data[1] = hCurrentRefq >> 8 & 0xFF;
    msgToSend.data[2] = hCurrentRefd & 0xFF;
    msgToSend.data[3] = hCurrentRefd >> 8 & 0xFF;
    msgToSend.data[4] = hCurrentMeasq & 0xFF;
    msgToSend.data[5] = hCurrentMeasq >> 8 & 0xFF;
    msgToSend.data[6] = hCurrentMeasd & 0xFF;
    msgToSend.data[7] = hCurrentMeasd >> 8 & 0xFF;
    
    //Send CAN message
    uCAL_CAN_SendMsg(msgToSend);
}

/* Function for sending the reference and measured speed */
void CANLOG_getSpeed(VCI_Handle_t * pHandle, uint8_t bMotorSelection)
{
    can_frame_t msgToSend;
    // Get speed values
    int16_t speedRef = MDI_GetMecSpeedRefUnit(pHandle->pPowertrain->pMDI,bMotorSelection);
    int16_t speedMeas = MDI_GetAvrgMecSpeedUnit(pHandle->pPowertrain->pMDI,bMotorSelection);
    // Prepare Header of CAN message 
    if(bMotorSelection == M1)
        msgToSend.id = CAN_ID_SPEED_M1;
	else
		msgToSend.id = CAN_ID_SPEED_M2;
    msgToSend.id_mode = CAN_ID_MODE_STANDARD;
    msgToSend.type    = CAN_FRAME_TYPE_DATA;
    msgToSend.data_length_code = 4;
    // Load data buffer
    msgToSend.data[0] = speedRef & 0xFF;
    msgToSend.data[1] = speedRef >> 8 & 0xFF;
    msgToSend.data[2] = speedMeas & 0xFF;
    msgToSend.data[3] = speedMeas >> 8 & 0xFF;
    
    //Send CAN message
    uCAL_CAN_SendMsg(msgToSend);
}

/* Function for sending the measured temperature */
void CANLOG_SendTemperature(VCI_Handle_t * pHandle, uint8_t bMotorSelection)
{
    can_frame_t msgToSend;
    // TO DO : Add a function in MDI module to get the motor temperature
    int16_t temperature = 70;
    
    // Prepare Header of CAN message 
    if(bMotorSelection == M1)
        msgToSend.id = CAN_ID_TEMPERATURE_M1;
	else
		msgToSend.id = CAN_ID_TEMPERATURE_M2;
    msgToSend.id_mode = CAN_ID_MODE_STANDARD;
    msgToSend.type    = CAN_FRAME_TYPE_DATA;
    msgToSend.data_length_code = 2;
    // Load data buffer
    msgToSend.data[0] = temperature & 0xFF;
    msgToSend.data[1] = temperature >> 8 & 0xFF;
    
    //Send CAN message
    uCAL_CAN_SendMsg(msgToSend);
}

/* Function for sending the throttle value and the brake status */
void CANLOG_SendThrottleBrake(VCI_Handle_t * pHandle)
{
    can_frame_t msgToSend;
    uint32_t throttleBrake = (uint32_t)BRK_IsPressed(pHandle->pPowertrain->pBrake) << 16 | Throttle_GetAvThrottleValue(pHandle->pPowertrain->pThrottle);
    
    // Prepare Header of CAN message 
    msgToSend.id = CAN_ID_THROTTLE_BRAKE;
    msgToSend.id_mode = CAN_ID_MODE_STANDARD;
    msgToSend.type    = CAN_FRAME_TYPE_DATA;
    msgToSend.data_length_code = 3;
    // Load data buffer
    msgToSend.data[0] = throttleBrake & 0xFF;
    msgToSend.data[1] = (throttleBrake >> 8)  & 0xFF;
    msgToSend.data[2] = (throttleBrake >> 16) & 0xFF;
    
    //Send CAN message
    uCAL_CAN_SendMsg(msgToSend);
}
#endif
