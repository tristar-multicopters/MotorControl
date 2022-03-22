/**
  ******************************************************************************
  * @file    md_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control a motor drive.
	******************************************************************************
*/

#include "md_interface.h"

void MDI_Init(MDI_Handle_t * pHandle)
{
	md_comm_init(pHandle->pMDC);
}

int16_t MDI_getBusVoltage(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDMeas.bus_voltage_mes;
}

int16_t MDI_getIq(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDMeas.iq_mes;
}

int16_t MDI_getId(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDMeas.id_mes;
}

int16_t MDI_getVq(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return 0; //TODO
}

int16_t MDI_getVd(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return 0; //TODO
}

int32_t MDI_getSpeed(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDMeas.speed_mes;
}

int16_t MDI_getAngle(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return 0;
}

int16_t MDI_getMotorTemp(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDMeas.temp_motor;
}

int16_t MDI_getHeatsinkTemp(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDMeas.temp_hs;
}

uint8_t MDI_getState(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDStateMachine.bMState;
}

uint16_t MDI_getCurrentFaults(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDStateMachine.hMFaultNow;
}

uint16_t MDI_getOccurredFaults(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return pHandle->pMDC->pMD[MotorSelection]->MDStateMachine.hMFaultOccurred;
}
	
bool MDI_StartMotor(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return md_startMotor(MotorSelection);
}	
	
bool MDI_StopMotor(MDI_Handle_t* pHandle, uint8_t MotorSelection)
{
	return md_stopMotor(MotorSelection);
}	
	
bool MDI_SetTorqueRamp(MDI_Handle_t* pHandle, uint8_t motorSelection, int16_t torque, int16_t duration)
{
	return md_setTorqueRamp(motorSelection, torque, duration);
}	
	
bool MDI_SetCurrentRef(MDI_Handle_t* pHandle, uint8_t motorSelection, int16_t iq, int16_t id)
{
	return md_setCurrentRef(motorSelection, iq, id);
}	
	
bool MDI_SetSpeedRamp(MDI_Handle_t* pHandle, uint8_t motorSelection, int32_t speed, int16_t duration)
{
	return md_setSpeedRamp(motorSelection, speed, duration);
}	
	
bool MDI_FaultAcknowledged(MDI_Handle_t* pHandle, uint8_t motorSelection)
{
	return md_faultAcknowledged(motorSelection);
}