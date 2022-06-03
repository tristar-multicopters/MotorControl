/**
  ******************************************************************************
  * @file    canbus_management.c
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module send messages to the CAN logger for diagnostics
  *
	******************************************************************************
*/

#include "canbus_management.h"

/** @brief	 Function for initialise the CAN instance
		@p 			 pHandle 		Handle of the CAN instance
		@return  Ongoing transfert flag state
*/

uint8_t CAN_Init(MCP25625_Handle_t * pCANHandle)
{
	return MCP25625_Init(pCANHandle);
}

/** @brief  Function for sending the vehicle status
		@p 			id     	CAN message ID
		@p 			status  Vehicle status
*/
void CAN_SendStatus(MCP25625_Handle_t * pCANHandle, VCI_Handle_t* pVCHandle, uint8_t motorSelection)
{
	int32_t status;
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 3,
	};
	
	if(motorSelection == M_NONE)
	{
		message.id = CAN_ID_STATUS_VC;
		status = VCSTM_GetState(pVCHandle->pStateMachine) << 16 |
						 VCSTM_GetFaultState(pVCHandle->pStateMachine);
	}
	
	else if(motorSelection == M1)
	{
		message.id = CAN_ID_STATUS_M1;
		status = MDI_getState(pVCHandle->pDrivetrain->pMDI, motorSelection)                |
						 MDI_getCurrentFaults(pVCHandle->pDrivetrain->pMDI, motorSelection)  <<  8 |
						 MDI_getOccurredFaults(pVCHandle->pDrivetrain->pMDI, motorSelection) << 16;
	}
	else
	{
		message.id = CAN_ID_STATUS_M2;
		status = MDI_getState(pVCHandle->pDrivetrain->pMDI, motorSelection)                |
						 MDI_getCurrentFaults(pVCHandle->pDrivetrain->pMDI, motorSelection)  <<  8 |
						 MDI_getOccurredFaults(pVCHandle->pDrivetrain->pMDI, motorSelection) << 16;
	}
	
	message.data[0] = status & 0xFF;         // Fault occurred
	message.data[1] = (status >> 8)  & 0xFF; // Fault now
	message.data[2] = (status >> 16) & 0xFF; // State
	MCP25625_WriteCANmsg(pCANHandle, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the bus voltage value
		@p 			id     	  CAN message ID
		@p 			bVoltage  Bus Voltage
*/
void CAN_SendVbus(MCP25625_Handle_t * pCANHandle, VCI_Handle_t* pVCHandle)
{
	uint8_t bVoltage = MDI_getBusVoltage(pVCHandle->pDrivetrain->pMDI,M1);
	CAN_Message_t message =
	{
		.id = CAN_ID_VBUS,
		.ext = 0,
		.rtr = 0,
		.length = 1
	};
	
	message.data[0] = bVoltage;
	MCP25625_WriteCANmsg(pCANHandle, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the current values
		@p 			id     	  CAN message ID  	
		@p 			currents  Values of id_meas, iq_meas, id_ref and iq_ref 
*/
void CAN_SendCurrent(MCP25625_Handle_t * pCANHandle, VCI_Handle_t* pVCHandle, uint8_t motorSelection)
{
	qd_t current_meas ;
	current_meas.q = MDI_getIq(pVCHandle->pDrivetrain->pMDI,motorSelection);
	current_meas.d = MDI_getId(pVCHandle->pDrivetrain->pMDI,motorSelection);
	
  qd_t current_ref  = {0, 0};
	
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 8
	};
	
	if(motorSelection == M1)
		message.id = CAN_ID_CURRENT_M1;
	else
		message.id = CAN_ID_CURRENT_M2;
	
	message.data[0] = current_ref.q & 0xFF;
	message.data[1] = current_ref.q  >> 8 & 0xFF; 
	message.data[2] = current_ref.d  & 0xFF;
	message.data[3] = current_ref.d  >> 8 & 0xFF;
	message.data[4] = current_meas.q & 0xFF;
	message.data[5] = current_meas.q >> 8 & 0xFF;
	message.data[6] = current_meas.d & 0xFF;
	message.data[7] = current_meas.d >> 8 & 0xFF;
	
	MCP25625_WriteCANmsg(pCANHandle, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the speed values
		@p 			id     	  CAN message ID  	
		@p 			speeds  Values of speed_meas and speed_ref 
*/
void CAN_SendSpeed(MCP25625_Handle_t * pCANHandle, VCI_Handle_t* pVCHandle, uint8_t motorSelection)
{	
	int32_t	speed_ref  = 0; 
	int32_t	speed_meas = MDI_getSpeed(pVCHandle->pDrivetrain->pMDI, motorSelection);
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 8
	};
	
	if(motorSelection == M1)
		message.id = CAN_ID_SPEED_M1;
		
	else
		message.id = CAN_ID_SPEED_M2;
		
	message.data[0] = speed_ref & 0xFF;
	message.data[1] = speed_ref  >> 8  & 0xFF; 
	message.data[2] = speed_ref >> 16 & 0xFF;
	message.data[3] = speed_ref >> 24 & 0xFF;
	message.data[4] = speed_meas & 0xFF;
	message.data[5] = speed_meas >> 8  & 0xFF;
	message.data[6] = speed_meas >> 16 & 0xFF;
	message.data[7] = speed_meas >> 24 & 0xFF;
	
	MCP25625_WriteCANmsg(pCANHandle, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the motor drive temperature
		@p 			id     	CAN message ID
		@p 			temperature  Motor drive temperature 
*/
void CAN_SendTemperature(MCP25625_Handle_t * pCANHandle, VCI_Handle_t* pVCHandle, uint8_t motorSelection)
{
	uint16_t temperature = MDI_getHeatsinkTemp(pVCHandle->pDrivetrain->pMDI, motorSelection);
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 2,
	};
	
	if(motorSelection == M1)
		message.id = CAN_ID_TEMPERATURE_M1;
	else
		message.id = CAN_ID_TEMPERATURE_M2;
	
	message.data[0] = temperature & 0xFF;
	message.data[1] = (temperature >> 8)  & 0xFF;
	
	MCP25625_WriteCANmsg(pCANHandle, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the throttle value and the brake status
		@p 			id     	CAN message ID
		@p 			throttleBrake  Throttle value concatenated with the brake status 
*/
void CAN_SendThrottleBrake(MCP25625_Handle_t * pCANHandle, VCI_Handle_t* pVCHandle)
{
	uint32_t throttle_brake = BRK_IsPressed(pVCHandle->pDrivetrain->pBrake) << 16 | THRO_GetAvThrottleValue(pVCHandle->pDrivetrain->pThrottle);
	CAN_Message_t message =
	{
		.id = CAN_ID_THROTTLE_BRAKE,
		.ext = 0,
		.rtr = 0,
		.length = 3
	};
	
	message.data[0] = throttle_brake & 0xFF;
	message.data[1] = (throttle_brake >> 8)  & 0xFF;
	message.data[2] = (throttle_brake >> 16)  & 0xFF;
	
	MCP25625_WriteCANmsg(pCANHandle, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for getting MCP error flag state.
		@return MCP_QUEUE_FULL if CAN queue is full
*/
bool CAN_IsQueueFull(MCP25625_Handle_t * pCANHandle)
{
	//todo: implementation
	
	if(pCANHandle->hError)
		return true;
	else
		return false;
}
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// FOR TESTING PURPOSES /////////////////////////////////////

void CAN_SendDummyMsg(MCP25625_Handle_t * pCANHandle)
{
	CAN_Message_t message =
	{
		.id = (uint32_t)0x299,
		.ext = 0,
		.rtr = 0,
		.length = 8,
		.data = {1,2,3,4,5,6,7,8},
	};
	
	for(int i = 0; i < 64; i++)
	{
		message.id++;
		MCP25625_WriteCANmsg(pCANHandle, CAN_TX_BUFFER_0, &message);
	}
}

