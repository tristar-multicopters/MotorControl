/**
  ******************************************************************************
  * @file    canbus_management.h
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module send messages to the CAN logger for diagnostics
  *
	******************************************************************************
	*/
	
#ifndef __CANBUS_MANAGEMENT_H
#define __CANBUS_MANAGEMENT_H

#include "mcp25625_comm.h"
#include "mc_defines.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/****************** FOR  CANBUS LOGGER ******************/
#define CAN_SIMPLE_TIME_TICK
typedef struct
{
	MCP25625_Handle_t * pMCP;
	uint8_t lock_pin;
	MCP_CAN_id_t current_ID_msg;
	bool isLoadingTXbuff;
} CAN_Handle_t;

/************************************ FUNCTIONS *************************************/


uint8_t CAN_Init(CAN_Handle_t* pHandle);
void CAN_SendStatus(MCP_CAN_id_t id, int32_t status);
void CAN_SendVbus(MCP_CAN_id_t id, uint8_t bVoltage);
void CAN_SendCurrent(MCP_CAN_id_t id, int16_t iq_ref, int16_t id_ref, int16_t iq_meas, int16_t id_meas);
void CAN_SendSpeed(MCP_CAN_id_t id, int32_t speed_ref, int32_t speed_meas);
void CAN_SendTemperature(MCP_CAN_id_t id, uint16_t temperature);
void CAN_SendThrottleBrake(MCP_CAN_id_t id, uint32_t throttleBrake);
void CAN_ProcessRXMsg(CAN_Message_t message);

#endif
