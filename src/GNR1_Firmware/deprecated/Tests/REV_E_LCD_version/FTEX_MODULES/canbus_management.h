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

#include "mc_defines.h"
#include "vc_interface.h"
#include "mcp25625_comm.h"

typedef enum
{
	CAN_ID_STATUS_VC      = 0x1,
  CAN_ID_THROTTLE_BRAKE = 0x2,
	CAN_ID_VBUS					  = 0x10,
	
	CAN_ID_STATUS_M1 		  = 0xA0,
  CAN_ID_CURRENT_M1			= 0x20,
	CAN_ID_SPEED_M1				= 0x30,
	CAN_ID_TEMPERATURE_M1	= 0x40,
	
	CAN_ID_STATUS_M2	    = 0xA1,
	CAN_ID_CURRENT_M2	    = 0x21,
	CAN_ID_SPEED_M2       = 0x31,
	CAN_ID_TEMPERATURE_M2 = 0x41,
	
}	VC_CAN_id_t;

/************************************ FUNCTIONS *************************************/

/* Task function for managing the CAN instance */
void TSK_CANmsg(void * pvParameter);

/* Function for initialise the CAN instance*/
uint8_t CAN_Init(MCP25625_Handle_t * pCANHandle);

/* Function for sending the vehicle status */
void CAN_SendStatus(MCP25625_Handle_t * pCANHandle, VC_Handle_t* pVCHandle, uint8_t motorSelection);

/* Function for sending the bus voltage value*/
void CAN_SendVbus(MCP25625_Handle_t * pCANHandle, VC_Handle_t* pVCHandle);

/* Function for sending the currents values */
void CAN_SendCurrent(MCP25625_Handle_t * pCANHandle, VC_Handle_t* pVCHandle, uint8_t motorSelection);

/* Function for sending the motor speed*/
void CAN_SendSpeed(MCP25625_Handle_t * pCANHandle, VC_Handle_t* pVCHandle, uint8_t motorSelection);

/* Function for sending the motor drive temperature */
void CAN_SendTemperature(MCP25625_Handle_t * pCANHandle, VC_Handle_t* pVCHandle, uint8_t motorSelection);

/* Function for sending the vehicle throttle value and the brake status*/
void CAN_SendThrottleBrake(MCP25625_Handle_t * pCANHandle, VC_Handle_t* pVCHandle);

/* Function for knowing if CAN message queue is full */
bool CAN_queueIsFull( MCP25625_Handle_t * pCANHandle );
	
/******************* Dummy Function for testing ******************************/
void CAN_SendDummyMsg( MCP25625_Handle_t * pCANHandle );
#endif
