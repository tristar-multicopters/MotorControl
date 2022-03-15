/**
  ******************************************************************************
  * @file    vc_config.h
  * @author  Jorge Polo, FTEX
  * @brief   Module that has functions for getting (and setting) the 
  *          values of the motor drive parameters
	******************************************************************************
*/

#include "md_interface.h"

void MD_setMode(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDRTParam.mode = (value & 0xFF);
}

void MD_setStatus(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDStateMachine.bMState = (MC_State_t)(value & 0xFF);
}

void MD_setTorqueKp(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDRTParam.torque_kp = value;
}

void MD_setTorqueKi(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDRTParam.torque_ki = value;
}

void MD_setFluxKp(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDRTParam.flux_kp = value;
}	

void MD_setFluxKi(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDRTParam.flux_ki = value;
}

void MD_setSpeedKp(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDRTParam.speed_kp = value;
}

void MD_setSpeedKi(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDRTParam.speed_ki = value;
}

void MD_setBusVoltage(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDMeas.bus_voltage_mes = value;
}

void MD_setIqMeas(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDMeas.iq_mes = value;
}

void MD_setIdMeas(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDMeas.id_mes = value;
}

void MD_setMotorTemp(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDMeas.temp_u = value;
}

void MD_setSpeedMeas(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDMeas.speed_mes = value;
}

void MD_setSpeedRef(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDRTParam.speed_ref = value;
}

void MD_setFlags(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value)
{
	p_Handle->pMD[motorselection].MDStateMachine.hMFaultNow = value & 0xFF;
	p_Handle->pMD[motorselection].MDStateMachine.hMFaultOccurred = (value >> 8) & 0xFF;
}
