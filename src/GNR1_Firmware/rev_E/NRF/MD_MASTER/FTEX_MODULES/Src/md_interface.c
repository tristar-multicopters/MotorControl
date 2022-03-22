/**
  ******************************************************************************
  * @file    md_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control a motor drive.
	******************************************************************************
*/

#include "md_interface.h"

void MDI_Init(MDI_Handle_t * pHandle)
{}

int16_t MDI_getBusVoltage(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

int16_t MDI_getIq(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

int16_t MDI_getId(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

int16_t MDI_getVq(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

int16_t MDI_getVd(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

int16_t MDI_getSpeed(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

int16_t MDI_getAngle(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

int16_t MDI_getMotorTemp(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

int16_t MDI_getHeatsinkTemp(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

uint8_t MDI_getState(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

uint16_t MDI_getCurrentFaults(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}

uint16_t MDI_getOccurredFaults(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}
	
uint8_t MDI_StartMotor(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}	
	
uint8_t MDI_StopMotor(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{}	
	
uint8_t MDI_SetTorqueRamp(MDI_Handle_t* pHandle, uint8_t motorSelection, int16_t torque, int16_t duration)
{}	
	
uint8_t MDI_SetCurrentRef(MDI_Handle_t* pHandle, uint8_t motorSelection, int16_t iq, int16_t id)
{}	
	
uint8_t MDI_SetSpeedRamp(MDI_Handle_t* pHandle, uint8_t motorSelection, int32_t speed, int16_t duration)
{}	
	
uint8_t MDI_FaultAcknowledged(MDI_Handle_t* pHandle, uint8_t motorSelection)
{}