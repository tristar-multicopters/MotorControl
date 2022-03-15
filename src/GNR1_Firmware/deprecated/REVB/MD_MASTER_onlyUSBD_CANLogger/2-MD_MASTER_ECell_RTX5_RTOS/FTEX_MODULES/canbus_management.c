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

uint8_t CAN_Init(VC_Handle_t* pHandle)
{
	return MCP25625_Init(pHandle->pMCP);
}

/** @brief  Function for sending the vehicle status
		@p 			id     	CAN message ID
		@p 			status  Vehicle status
*/
void CAN_SendStatus(VC_Handle_t* pHandle, uint8_t motorSelection)
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
		message.id = CAN_STATUS_VC_ID;
		status = VC_getVehicleState(pHandle)    << 16 |
						 VC_getVehicleFaultNow(pHandle) << 8  |
						 VC_getVehicleFaultOccurred(pHandle);
	}
	
	else if(motorSelection == M1)
	{
		message.id = CAN_STATUS_M1_ID;
		status = VC_getMotorState(pHandle, motorSelection)              |
						 VC_getMotorFaultNow(pHandle, motorSelection)      << 8 |
						 VC_getMotorFaultOccurred(pHandle, motorSelection) << 16;
	}
	else
	{
		message.id = CAN_STATUS_M2_ID;
		status = VC_getMotorState(pHandle, motorSelection)              |
						 VC_getMotorFaultNow(pHandle, motorSelection)      << 8 |
						 VC_getMotorFaultOccurred(pHandle, motorSelection) << 16;
	}
	
	message.data[0] = status & 0xFF;         // Fault occurred
	message.data[1] = (status >> 8)  & 0xFF; // Fault now
	message.data[2] = (status >> 16) & 0xFF; // State
	MCP25625_WriteCANmsg(pHandle->pMCP, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the bus voltage value
		@p 			id     	  CAN message ID
		@p 			bVoltage  Bus Voltage
*/
void CAN_SendVbus(VC_Handle_t* pHandle)
{
	uint8_t bVoltage = VC_getBattVoltage(pHandle, M1);
	CAN_Message_t message =
	{
		.id = CAN_VBUS_ID,
		.ext = 0,
		.rtr = 0,
		.length = 1
	};
	
	message.data[0] = bVoltage;
	MCP25625_WriteCANmsg(pHandle->pMCP, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the current values
		@p 			id     	  CAN message ID  	
		@p 			currents  Values of id_meas, iq_meas, id_ref and iq_ref 
*/
void CAN_SendCurrent(VC_Handle_t* pHandle, uint8_t motorSelection)
{
	qd_t current_meas = VC_getMotorIqIdMeas(pHandle, motorSelection);
	qd_t current_ref  = VC_getMotorIqIdRef(pHandle, motorSelection);
	
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 8
	};
	
	if(motorSelection == M1)
		message.id = CAN_CURRENT_M1_ID;
	else
		message.id = CAN_CURRENT_M2_ID;
	
	message.data[0] = current_ref.q & 0xFF;
	message.data[1] = current_ref.q  >> 8 & 0xFF; 
	message.data[2] = current_ref.d  & 0xFF;
	message.data[3] = current_ref.d  >> 8 & 0xFF;
	message.data[4] = current_meas.q & 0xFF;
	message.data[5] = current_meas.q >> 8 & 0xFF;
	message.data[6] = current_meas.d & 0xFF;
	message.data[7] = current_meas.d >> 8 & 0xFF;
	
	MCP25625_WriteCANmsg(pHandle->pMCP, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the speed values
		@p 			id     	  CAN message ID  	
		@p 			speeds  Values of speed_meas and speed_ref 
*/
void CAN_SendSpeed(VC_Handle_t* pHandle, uint8_t motorSelection)
{	
	int32_t	speed_ref  = VC_getMotorSpeedRef(pHandle, motorSelection); 
	int32_t	speed_meas = VC_getMotorSpeedMeas(pHandle, motorSelection);
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 8
	};
	
	if(motorSelection == M1)
		message.id = CAN_SPEED_M1_ID;
		
	else
		message.id = CAN_SPEED_M2_ID;
		
	message.data[0] = speed_ref & 0xFF;
	message.data[1] = speed_ref  >> 8  & 0xFF; 
	message.data[2] = speed_ref >> 16 & 0xFF;
	message.data[3] = speed_ref >> 24 & 0xFF;
	message.data[4] = speed_meas & 0xFF;
	message.data[5] = speed_meas >> 8  & 0xFF;
	message.data[6] = speed_meas >> 16 & 0xFF;
	message.data[7] = speed_meas >> 24 & 0xFF;
	
	MCP25625_WriteCANmsg(pHandle->pMCP, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the motor drive temperature
		@p 			id     	CAN message ID
		@p 			temperature  Motor drive temperature 
*/
void CAN_SendTemperature(VC_Handle_t* pHandle, uint8_t motorSelection)
{
	uint16_t temperature = pHandle->pMDComm->pMD[motorSelection].MDMeas.temp_u;
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 2,
	};
	
	if(motorSelection == M1)
		message.id = CAN_TEMPERATURE_M1_ID;
	else
		message.id = CAN_TEMPERATURE_M2_ID;
	
	message.data[0] = temperature & 0xFF;
	message.data[1] = (temperature >> 8)  & 0xFF;
	
	MCP25625_WriteCANmsg(pHandle->pMCP, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for sending the throttle value and the brake status
		@p 			id     	CAN message ID
		@p 			throttleBrake  Throttle value concatenated with the brake status 
*/
void CAN_SendThrottleBrake(VC_Handle_t* pHandle)
{
	uint32_t throttle_brake = VC_isBrakeOn(pHandle) << 16 | VC_getThrottle(pHandle);
	CAN_Message_t message =
	{
		.id = CAN_THROTTLE_ID,
		.ext = 0,
		.rtr = 0,
		.length = 3
	};
	
	message.data[0] = throttle_brake & 0xFF;
	message.data[1] = (throttle_brake >> 8)  & 0xFF;
	message.data[2] = (throttle_brake >> 16)  & 0xFF;
	
	MCP25625_WriteCANmsg(pHandle->pMCP, CAN_TX_BUFFER_0, &message);
}

/** @brief  Function for getting MCP error flag state.
		@return MCP_QUEUE_FULL if CAN queue is full
*/
bool CAN_queueIsFull(VC_Handle_t* pHandle)
{
	if(pHandle->pMCP->hError)
		return true;
	else
		return false;
}
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// FOR TESTING PURPOSES /////////////////////////////////////

void CAN_SendDummyMsg(VC_Handle_t* pHandle)
{
	CAN_Message_t message =
	{
		.id = (MCP_CAN_id_t)0x299,
		.ext = 0,
		.rtr = 0,
		.length = 8,
		.data = {1,2,3,4,5,6,7,8},
	};
	
	for(int i = 0; i < 64; i++)
	{
		message.id++;
		MCP25625_WriteCANmsg(pHandle->pMCP, CAN_TX_BUFFER_0, &message);
	}
}
//void CAN_LowVWarning(uint8_t voltage)
//{
//	
//}
