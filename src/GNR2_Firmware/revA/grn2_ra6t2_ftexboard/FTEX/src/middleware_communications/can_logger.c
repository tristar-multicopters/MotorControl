/**
*  can_logger.c
*  Middleware layer for managing CAN logging messages
*/
#include "can_logger.h"

//================================= PRIVATE MEMBERS =================================//

//========================== PRIVATE FUNCTION PROTOTYPES ============================//




//========================== PRIVATE FUNCTION DEFINITIONS ============================//

/* Function for sending the vehicle status */
void CANLog_SendStatus(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * pHandle, uint8_t bMotorSelection)
{
    CO_IF_FRM msgToSend;
    uint32_t status = 0;
    // Prepare Header of CAN message 
    msgToSend.DLC = 3;
    // Get status, fault now and occured faults
	if(bMotorSelection == M_NONE)
	{
		msgToSend.Identifier = CAN_ID_STATUS_VC;
		status = ((uint32_t)VCSTM_GetState(pHandle->pStateMachine) << 16) |
				  (uint32_t)VCSTM_GetFaultState(pHandle->pStateMachine);
        // Load data buffer
        msgToSend.Data[0] = status & 0xFF;     // Fault occurred
        msgToSend.Data[1] = (status >> 8)  & 0xFF; // Fault now
        msgToSend.Data[2] = (status >> 16) & 0xFF; // State
        
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
	}
	
	else if(bMotorSelection == M1)
	{
		msgToSend.Identifier = CAN_ID_STATUS_M1;
		status = MDI_GetSTMState(pHandle->pPowertrain->pMDI, bMotorSelection)                |
						 (uint32_t)MDI_GetCurrentFaults(pHandle->pPowertrain->pMDI, bMotorSelection)  <<  8 |
						 (uint32_t)MDI_GetOccurredFaults(pHandle->pPowertrain->pMDI, bMotorSelection) << 16;
        // Load data buffer
        msgToSend.Data[0] = status & 0xFF;     // Fault occurred
        msgToSend.Data[1] = (status >> 8)  & 0xFF; // Fault now
        msgToSend.Data[2] = (status >> 16) & 0xFF; // State
        
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
	}
	else if(bMotorSelection == M2)
	{
		msgToSend.Identifier = CAN_ID_STATUS_M2;
		status = MDI_GetSTMState(pHandle->pPowertrain->pMDI, bMotorSelection)                |
						 (uint32_t)MDI_GetCurrentFaults(pHandle->pPowertrain->pMDI, bMotorSelection)  <<  8 |
						 (uint32_t)MDI_GetOccurredFaults(pHandle->pPowertrain->pMDI, bMotorSelection) << 16;
        // Load data buffer
        msgToSend.Data[0] = status & 0xFF;     // Fault occurred
        msgToSend.Data[1] = (status >> 8)  & 0xFF; // Fault now
        msgToSend.Data[2] = (status >> 16) & 0xFF; // State
        
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
	}

}

/* Function for sending the bus voltage value*/
void CANLog_SendVbus(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t* pHandle)
{
    // TO DO: Add a function in the MDI module for getting the Bus voltage
    (void) pHandle;
    
    CO_IF_FRM msgToSend;
    
    // Prepare Header of CAN message 
    uint8_t bVoltage = 0x64;
    msgToSend.Identifier = CAN_ID_VBUS;
    msgToSend.DLC = 1;
    // Load data buffer
    msgToSend.Data[0] = bVoltage;
    
    //Send CAN message
    pCANOpenCANInterface->Send(&msgToSend);
}

/* Function for sending the reference and measured currents */
void CANLog_SendCurrent(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * pHandle, uint8_t bMotorSelection)
{
    CO_IF_FRM msgToSend;
    msgToSend.DLC = 8;

    // Prepare Header of CAN message 
    if(bMotorSelection == M1)
    {
        msgToSend.Identifier = CAN_ID_CURRENT_M1;
        // Get current values
        int16_t hCurrentRefq = MDI_GetIqdref(pHandle->pPowertrain->pMDI, bMotorSelection).q;
        int16_t hCurrentRefd = MDI_GetIqdref(pHandle->pPowertrain->pMDI, bMotorSelection).d;
        int16_t hCurrentMeasq = MDI_GetIqd(pHandle->pPowertrain->pMDI, bMotorSelection).q;
        int16_t hCurrentMeasd = MDI_GetIqd(pHandle->pPowertrain->pMDI, bMotorSelection).d;
        // Load data buffer
        msgToSend.Data[0] = hCurrentRefq & 0xFF;
        msgToSend.Data[1] = hCurrentRefq >> 8 & 0xFF;
        msgToSend.Data[2] = hCurrentRefd & 0xFF;
        msgToSend.Data[3] = hCurrentRefd >> 8 & 0xFF;
        msgToSend.Data[4] = hCurrentMeasq & 0xFF;
        msgToSend.Data[5] = hCurrentMeasq >> 8 & 0xFF;
        msgToSend.Data[6] = hCurrentMeasd & 0xFF;
        msgToSend.Data[7] = hCurrentMeasd >> 8 & 0xFF;
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
    }
	else
    {
		msgToSend.Identifier = CAN_ID_CURRENT_M2;
        // Get current values
        int16_t hCurrentRefq = MDI_GetIqdref(pHandle->pPowertrain->pMDI, bMotorSelection).q;
        int16_t hCurrentRefd = MDI_GetIqdref(pHandle->pPowertrain->pMDI, bMotorSelection).d;
        int16_t hCurrentMeasq = MDI_GetIqd(pHandle->pPowertrain->pMDI, bMotorSelection).q;
        int16_t hCurrentMeasd = MDI_GetIqd(pHandle->pPowertrain->pMDI, bMotorSelection).d;
        // Load data buffer
        msgToSend.Data[0] = hCurrentRefq & 0xFF;
        msgToSend.Data[1] = hCurrentRefq >> 8 & 0xFF;
        msgToSend.Data[2] = hCurrentRefd & 0xFF;
        msgToSend.Data[3] = hCurrentRefd >> 8 & 0xFF;
        msgToSend.Data[4] = hCurrentMeasq & 0xFF;
        msgToSend.Data[5] = hCurrentMeasq >> 8 & 0xFF;
        msgToSend.Data[6] = hCurrentMeasd & 0xFF;
        msgToSend.Data[7] = hCurrentMeasd >> 8 & 0xFF;
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
    }

}

/* Function for sending the reference and measured speed */
void CANLog_SendSpeed(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * pHandle, uint8_t bMotorSelection)
{
    CO_IF_FRM msgToSend;

    if (bMotorSelection == M1)
    {
        // Get speed values
        int16_t speedRef = MDI_GetMecSpeedRefUnit(pHandle->pPowertrain->pMDI,bMotorSelection);
        int16_t speedMeas = MDI_GetAvrgMecSpeedUnit(pHandle->pPowertrain->pMDI,bMotorSelection);
        msgToSend.Identifier = CAN_ID_SPEED_M1;
        msgToSend.DLC = 4;
        // Load data buffer
        msgToSend.Data[0] = speedRef & 0xFF;
        msgToSend.Data[1] = speedRef >> 8 & 0xFF;
        msgToSend.Data[2] = speedMeas & 0xFF;
        msgToSend.Data[3] = speedMeas >> 8 & 0xFF;
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
	}
    else if (bMotorSelection == M2)
    {
        int16_t speedRef = MDI_GetMecSpeedRefUnit(pHandle->pPowertrain->pMDI,bMotorSelection);
        int16_t speedMeas = MDI_GetAvrgMecSpeedUnit(pHandle->pPowertrain->pMDI,bMotorSelection);
		msgToSend.Identifier = CAN_ID_SPEED_M2;
        msgToSend.DLC = 4;
        // Load data buffer
        msgToSend.Data[0] = speedRef & 0xFF;
        msgToSend.Data[1] = speedRef >> 8 & 0xFF;
        msgToSend.Data[2] = speedMeas & 0xFF;
        msgToSend.Data[3] = speedMeas >> 8 & 0xFF;
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
    }
}

/* Function for sending the measured temperature */
void CANLog_SendTemperature(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * pHandle, uint8_t bMotorSelection)
{
    CO_IF_FRM msgToSend;
    
    // Prepare Header of CAN message 
    if(bMotorSelection == M1)
    {
        int16_t temperature = MCInterface_GetInverterTemp(pHandle->pPowertrain->pMDI->pMCI);
        
        // todo: support negative temperatures, in case of motor startup in cold weather
        // adding this assignment for now because MCInterface_GetInverterTemp used to return uint_16 and we must test
        // again now that it's returning a signed value
        if (temperature < 0)
        {
            temperature = 0;
        }
        
        msgToSend.Identifier = CAN_ID_TEMPERATURE_M1;
        msgToSend.DLC = 2;
        // Load data buffer
        msgToSend.Data[0] = temperature & 0xFF;
        msgToSend.Data[1] = temperature >> 8 & 0xFF;
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
    }
	else
    {
        // TO DO : Add a function in MDI module to get the motor temperature
        int16_t temperature = 70;
		msgToSend.Identifier = CAN_ID_TEMPERATURE_M2;
        msgToSend.DLC = 2;
        // Load data buffer
        msgToSend.Data[0] = temperature & 0xFF;
        msgToSend.Data[1] = temperature >> 8 & 0xFF;
        //Send CAN message
        pCANOpenCANInterface->Send(&msgToSend);
    }
}

/* Function for sending the throttle value and the brake status */
void CANLog_SendThrottleBrake(const CO_IF_CAN_DRV * pCANOpenCANInterface, VCI_Handle_t * pHandle)
{
    CO_IF_FRM msgToSend;
    uint32_t throttleBrake = (uint32_t)BRK_IsPressed(pHandle->pPowertrain->pBrake) << 16 | Throttle_GetAvThrottleValue(pHandle->pPowertrain->pThrottle);
    
    // Prepare Header of CAN message 
    msgToSend.Identifier = CAN_ID_THROTTLE_BRAKE;
    msgToSend.DLC = 3;
    // Load data buffer
    msgToSend.Data[0] = throttleBrake & 0xFF;
    msgToSend.Data[1] = (throttleBrake >> 8)  & 0xFF;
    msgToSend.Data[2] = (throttleBrake >> 16) & 0xFF;
    
    //Send CAN message
    pCANOpenCANInterface->Send(&msgToSend);
}

