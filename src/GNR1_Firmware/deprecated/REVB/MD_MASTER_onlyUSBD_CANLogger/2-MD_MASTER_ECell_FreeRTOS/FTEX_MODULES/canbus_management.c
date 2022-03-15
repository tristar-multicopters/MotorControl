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
static QueueHandle_t CANTX_queue;
TaskHandle_t TSK_CANmsgTX_handle_t;

uint8_t CAN_Init(CAN_Handle_t* pHandle)
{
	m_pCAN_handle = pHandle;
	pHandle->current_ID_msg = CAN_STATUS_VC_ID;
	pHandle->isLoadingTXbuff = false;
	// Configure pin for lock vehicle as output
	nrf_gpio_cfg_output(m_pCAN_handle->lock_pin);
	return MCP25625_Init(m_pCAN_handle->pMCP);
}
	
void CAN_SendStatus(MCP_CAN_id_t id, int32_t status)
{
	CAN_Message_t message =
	{
		.ext = 0,
		.rtr = 0,
		.length = 3,
	};

	message.id = id;
	message.data[0] = status & 0xFF;         // Fault occurred
	message.data[1] = (status >> 8)  & 0xFF; // Fault now
	message.data[2] = (status >> 16) & 0xFF; // State
	CAN_toSend_t toSend = MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
	
	if(xQueueSend(CANTX_queue, &toSend, 0))
	{
		xTaskNotifyGive(TSK_CANmsgTX_handle_t);
	}
}

void CAN_SendVbus(MCP_CAN_id_t id, uint8_t bVoltage)
{
	CAN_Message_t message =
	{
		.id = id,
		.ext = 0,
		.rtr = 0,
		.length = 1
	};
	
	message.data[0] = bVoltage;
	CAN_toSend_t toSend = MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
	
	if(xQueueSend(CANTX_queue, &toSend, 0))
	{
		xTaskNotifyGive(TSK_CANmsgTX_handle_t);
	}
}

void CAN_SendCurrent(MCP_CAN_id_t id, int16_t iq_ref, int16_t id_ref, int16_t iq_meas, int16_t id_meas)
{
	CAN_Message_t message =
	{
		.id = id,
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
	
	CAN_toSend_t toSend = MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
	
	if(xQueueSend(CANTX_queue, &toSend, 0))
	{
		xTaskNotifyGive(TSK_CANmsgTX_handle_t);
	}
}

void CAN_SendSpeed(MCP_CAN_id_t id, int32_t speed_ref, int32_t speed_meas)
{
	CAN_Message_t message =
	{
		.id = id,
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
	
	CAN_toSend_t toSend = MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
	
	if(xQueueSend(CANTX_queue, &toSend, 0))
	{
		xTaskNotifyGive(TSK_CANmsgTX_handle_t);
	}
}

void CAN_SendTemperature(MCP_CAN_id_t id, uint16_t temperature)
{
	CAN_Message_t message =
	{
		.id = id,
		.ext = 0,
		.rtr = 0,
		.length = 2,
	};
	
	message.data[0] = temperature & 0xFF;
	message.data[1] = (temperature >> 8)  & 0xFF;
	
	CAN_toSend_t toSend = MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
	
	if(xQueueSend(CANTX_queue, &toSend, 0))
	{
		xTaskNotifyGive(TSK_CANmsgTX_handle_t);
	}
}

void CAN_SendThrottleBrake(MCP_CAN_id_t id, uint32_t throttleBrake)
{
	CAN_Message_t message =
	{
		.id = id,
		.ext = 0,
		.rtr = 0,
		.length = 3
	};
	
	message.data[0] = throttleBrake & 0xFF;
	message.data[1] = (throttleBrake >> 8)  & 0xFF;
	message.data[2] = (throttleBrake >> 16)  & 0xFF;
	
	CAN_toSend_t toSend = MCP25625_WriteCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0, &message);
	
	if(xQueueSend(CANTX_queue, &toSend, 0))
	{
		xTaskNotifyGive(TSK_CANmsgTX_handle_t);
	}
}

static void CAN_assign_next_IDmsg()
{
	switch(m_pCAN_handle->current_ID_msg)
	{
		case CAN_STATUS_VC_ID:
			m_pCAN_handle->current_ID_msg = CAN_THROTTLE_ID;
			break;
		case CAN_THROTTLE_ID:
			m_pCAN_handle->current_ID_msg = CAN_VBUS_ID;
			break;
		case CAN_VBUS_ID:
			m_pCAN_handle->current_ID_msg = CAN_STATUS_M1_ID;
			break;
		case CAN_STATUS_M1_ID:
			m_pCAN_handle->current_ID_msg = CAN_CURRENT_M1_ID;
			break;
		case CAN_CURRENT_M1_ID:
			m_pCAN_handle->current_ID_msg = CAN_SPEED_M1_ID;
			break;
		case CAN_SPEED_M1_ID:
			m_pCAN_handle->current_ID_msg = CAN_TEMPERATURE_M1_ID;
			break;
		case CAN_TEMPERATURE_M1_ID:
			#ifndef USE_TWO_MOTORS
			m_pCAN_handle->current_ID_msg = CAN_STATUS_VC_ID;
			#else
			m_pCAN_handle->current_ID_msg = CAN_STATUS_M2_ID;
		  #endif
			break;
		
		case CAN_STATUS_M2_ID:
			m_pCAN_handle->current_ID_msg = CAN_CURRENT_M2_ID;
			break;		
		case CAN_CURRENT_M2_ID:
			m_pCAN_handle->current_ID_msg = CAN_SPEED_M2_ID;
			break;	
		case CAN_SPEED_M2_ID:
			m_pCAN_handle->current_ID_msg = CAN_TEMPERATURE_M2_ID;
			break;			
		case CAN_TEMPERATURE_M2_ID:
			m_pCAN_handle->current_ID_msg = CAN_STATUS_VC_ID;
			break;
		default:
			break;
	}
}	

void TSK_CANmsg(void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	CANTX_queue = xQueueCreate(MCP_CANMSG_SIZE, sizeof(CAN_toSend_t));
	
	while(true)
	{
		if (xTaskNotifyWait(0, 0, NULL, 50))
		{
			CAN_toSend_t send_can_message;
			if(xQueueReceive(CANTX_queue, & send_can_message, 0) == pdPASS)
			{
				if(!m_pCAN_handle->isLoadingTXbuff)
				{
					m_pCAN_handle->isLoadingTXbuff = true;
					MCP25625_LoadTxBuffer(m_pCAN_handle->pMCP, send_can_message.size, send_can_message.tx_buffer);
				}
			}
		}
		
		else if(m_pCAN_handle->isLoadingTXbuff)
		{
			m_pCAN_handle->isLoadingTXbuff = false;
			MCP25625_SendCANmsg(m_pCAN_handle->pMCP, CAN_TX_BUFFER_0);
		}
		
		else if(m_pCAN_handle->pMCP->IT_received)
		{
			MCP25625_manage_IT();
			if(m_pCAN_handle->pMCP->state != MCP_ERROR_MERRF &&
				 m_pCAN_handle->pMCP->state != MCP_ERROR_ERRIF)
			{
				CAN_assign_next_IDmsg();
			}
			else
			{
				MCP25625_check_errors();
			}
		}
	}
}

void CAN_ProcessRXMsg(CAN_Message_t message)
{
	switch(message.id)
	{
		case CAN_LOCK:
			if(message.data[0] == 1)
				nrf_gpio_pin_set(m_pCAN_handle->lock_pin);
			else if(message.data[0] == 2)
				nrf_gpio_pin_clear(m_pCAN_handle->lock_pin);
			break;
		default:
			break;
	}
	m_pCAN_handle->pMCP->ongoing_transfer = false;
}

//void CAN_LowVWarning(uint8_t voltage)
//{
//	
//}
