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

/****************** FOR  CANBUS LOGGER ******************/
#define CAN_SIMPLE_TIME_TICK

/************************************ FUNCTIONS *************************************/

/* Task function for managing the CAN instance */
void TSK_CANmsg(void * pvParameter);

/* Function for initialise the CAN instance*/
uint8_t CAN_Init(VC_Handle_t* pHandle);

/* Function for sending the vehicle status */
void CAN_SendStatus(VC_Handle_t* pHandle, uint8_t motorSelection);

/* Function for sending the bus voltage value*/
void CAN_SendVbus(VC_Handle_t* pHandle);

/* Function for sending the currents values */
void CAN_SendCurrent(VC_Handle_t* pHandle, uint8_t motorSelection);

/* Function for sending the motor speed*/
void CAN_SendSpeed(VC_Handle_t* pHandle, uint8_t motorSelection);

/* Function for sending the motor drive temperature */
void CAN_SendTemperature(VC_Handle_t* pHandle, uint8_t motorSelection);

/* Function for sending the vehicle throttle value and the brake status*/
void CAN_SendThrottleBrake(VC_Handle_t* pHandle);

/* Function for knowing if CAN message queue is full */
bool CAN_queueIsFull( VC_Handle_t* pHandle );
	
/******************* Dummy Function for testing ******************************/
void CAN_SendDummyMsg( VC_Handle_t* pHandle );
#endif
