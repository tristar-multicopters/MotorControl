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

/****************** FOR  CANBUS LOGGER ******************/

// ID for CAN messages
#define CAN_STATUS_VC_ID    0x1
#define CAN_THROTTLE_ID			0x2
#define CAN_STATUS_M_ID 		0xA0

#define CAN_VBUS_ID					0x10
#define CAN_CURRENT_ID			0x20
#define CAN_SPEED_ID				0x30
#define CAN_TEMPERATURE_ID	0x40


typedef struct
{
	MCP25625_Handle_t * pMCP;
} CAN_Handle_t;

/************************************ FUNCTIONS *************************************/

uint8_t CAN_Init(CAN_Handle_t* pHandle);
void CAN_SendStatus(uint8_t motorSelection, int32_t toSend);
void CAN_SendVbus(uint8_t motorSelection, uint8_t bVoltage);
void CAN_SendCurrent(uint8_t motorSelection, int16_t iq_ref, int16_t id_ref, int16_t iq_meas, int16_t id_meas);
void CAN_SendSpeed(uint8_t motorSelection, int32_t speed_ref, int32_t speed_meas);
void CAN_SendTemperature(uint8_t motorSelection, uint16_t temperature);
void CAN_SendThrottleBrake(uint8_t motorSelection, uint32_t throttleBrake);


#endif
