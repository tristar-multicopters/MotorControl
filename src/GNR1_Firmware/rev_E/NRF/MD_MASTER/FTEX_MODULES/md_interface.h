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

int16_t MDI_getBusVoltage(uint8_t MotorSelection);
int16_t MDI_getIq(uint8_t MotorSelection);
int16_t MDI_getId(uint8_t MotorSelection);
int16_t MDI_getVq(uint8_t MotorSelection);
int16_t MDI_getVd(uint8_t MotorSelection);
int16_t MDI_getSpeed(uint8_t MotorSelection);
int16_t MDI_getAngle(uint8_t MotorSelection);
int16_t MDI_getMotorTemp(uint8_t MotorSelection);
int16_t MDI_getHeatsinkTemp(uint8_t MotorSelection);
uint8_t MDI_getState(uint8_t MotorSelection);
uint16_t MDI_getFlags(uint8_t MotorSelection);

uint8_t MDI_StartMotor(uint8_t MotorSelection);
uint8_t MDI_StopMotor(uint8_t MotorSelection);
uint8_t MDI_SetTorqueRamp(uint8_t motorSelection, int16_t torque, int16_t duration);
uint8_t MDI_SetCurrentRef(uint8_t motorSelection, int16_t iq, int16_t id);
uint8_t MDI_SetSpeedRamp(uint8_t motorSelection, int32_t speed, int16_t duration);
uint8_t MDI_FaultAcknowledge(uint8_t motorSelection);


#endif
