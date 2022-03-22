/**
  ******************************************************************************
  * @file    md_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control a motor drive.
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MD_INTERFACE_H
#define __MD_INTERFACE_H

#include "md_comm.h"

typedef struct
{
	MD_Comm_Handle_t * pMDC;
} MDI_Handle_t;

void MDI_Init(MDI_Handle_t * pHandle);

int16_t MDI_getBusVoltage(MDI_Handle_t* pHandle, uint8_t MotorSelection);
int16_t MDI_getIq(MDI_Handle_t* pHandle, uint8_t MotorSelection);
int16_t MDI_getId(MDI_Handle_t* pHandle, uint8_t MotorSelection);
int16_t MDI_getVq(MDI_Handle_t* pHandle, uint8_t MotorSelection);
int16_t MDI_getVd(MDI_Handle_t* pHandle, uint8_t MotorSelection);
int32_t MDI_getSpeed(MDI_Handle_t* pHandle, uint8_t MotorSelection);
int16_t MDI_getAngle(MDI_Handle_t* pHandle, uint8_t MotorSelection);
int16_t MDI_getMotorTemp(MDI_Handle_t* pHandle, uint8_t MotorSelection);
int16_t MDI_getHeatsinkTemp(MDI_Handle_t* pHandle, uint8_t MotorSelection);
uint8_t MDI_getState(MDI_Handle_t* pHandle, uint8_t MotorSelection);
uint16_t MDI_getCurrentFaults(MDI_Handle_t* pHandle, uint8_t MotorSelection);
uint16_t MDI_getOccurredFaults(MDI_Handle_t* pHandle, uint8_t MotorSelection);

bool MDI_StartMotor(MDI_Handle_t* pHandle, uint8_t MotorSelection);
bool MDI_StopMotor(MDI_Handle_t* pHandle, uint8_t MotorSelection);
bool MDI_SetTorqueRamp(MDI_Handle_t* pHandle, uint8_t motorSelection, int16_t torque, int16_t duration);
bool MDI_SetCurrentRef(MDI_Handle_t* pHandle, uint8_t motorSelection, int16_t iq, int16_t id);
bool MDI_SetSpeedRamp(MDI_Handle_t* pHandle, uint8_t motorSelection, int32_t speed, int16_t duration);
void MDI_FaultAcknowledged(MDI_Handle_t* pHandle, uint8_t motorSelection);


#endif
