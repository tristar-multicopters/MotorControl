/**
  ******************************************************************************
  * @file    canbus_management.c
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module send messages to the CAN logger for diagnostics
  *
	******************************************************************************
*/

#include "canbus_management.h"

static CAN_Handle_t* m_pCAN_handle;


uint8_t CAN_Init(CAN_Handle_t* pHandle)
{
	m_pCAN_handle = pHandle;
	return MCP25625_Init(m_pCAN_handle->pMCP);
}
	
void CAN_SendStatus(uint8_t motorSelection, int32_t toSend)
{
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 3,
	};

	if(motorSelection == M_NONE)
		message.id = CAN_STATUS_VC_ID;
	else
		message.id = CAN_STATUS_M_ID | motorSelection;
	message.data[0] = toSend & 0xFF;         // Fault occurred
	message.data[1] = (toSend >> 8)  & 0xFF; // Fault now
	message.data[2] = (toSend >> 16) & 0xFF; // State
	
	MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
}

void CAN_SendVbus(uint8_t motorSelection, uint8_t bVoltage)
{
	CAN_Message_t message =
	{
		.id = CAN_VBUS_ID | motorSelection,
		.ext = 0,
		.rtr = 0,
		.length = 1
	};
	
	message.data[0] = bVoltage;
	MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
}

void CAN_SendCurrent(uint8_t motorSelection, int16_t iq_ref, int16_t id_ref, int16_t iq_meas, int16_t id_meas)
{
	CAN_Message_t message =
	{
		.id = CAN_CURRENT_ID | motorSelection,
		.ext = 0,
		.rtr = 0,
		.length = 8
	};
	
	message.data[0] = iq_ref & 0xFF;
	message.data[1] = iq_ref >> 8 & 0xFF; 
	message.data[2] = id_ref & 0xFF;
	message.data[3] = id_ref >> 8 & 0xFF;
	message.data[4] = iq_meas & 0xFF;
	message.data[5] = iq_meas >> 8 & 0xFF;
	message.data[6] = id_meas & 0xFF;
	message.data[7] = id_meas >> 8 & 0xFF;
	
	MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
}

void CAN_SendSpeed(uint8_t motorSelection, int32_t speed_ref, int32_t speed_meas)
{
	CAN_Message_t message =
	{
		.id = CAN_SPEED_ID | motorSelection,
		.ext = 0,
		.rtr = 0,
		.length = 8
	};
	
	message.data[0] = speed_ref & 0xFF;
	message.data[1] = speed_ref  >> 8  & 0xFF; 
	message.data[2] = speed_ref  >> 16 & 0xFF;
	message.data[3] = speed_ref  >> 24 & 0xFF;
	message.data[4] = speed_meas & 0xFF;
	message.data[5] = speed_meas >> 8  & 0xFF;
	message.data[6] = speed_meas >> 16 & 0xFF;
	message.data[7] = speed_meas >> 24 & 0xFF;
	
	MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
}

void CAN_SendTemperature(uint8_t motorSelection, uint16_t temperature)
{
	CAN_Message_t message =
	{
		.id = CAN_TEMPERATURE_ID | motorSelection,
		.ext = 0,
		.rtr = 0,
		.length = 2,
	};
	
	message.data[0] = temperature & 0xFF;
	message.data[1] = (temperature >> 8)  & 0xFF;
	
	MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
}

void CAN_SendThrottleBrake(uint8_t motorSelection, uint32_t throttleBrake)
{
	CAN_Message_t message =
	{
		.id = CAN_THROTTLE_ID | motorSelection,
		.ext = 0,
		.rtr = 0,
		.length = 3
	};
	
	message.data[0] = throttleBrake & 0xFF;
	message.data[1] = (throttleBrake >> 8)  & 0xFF;
	message.data[2] = (throttleBrake >> 16)  & 0xFF;
	
	MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
}
