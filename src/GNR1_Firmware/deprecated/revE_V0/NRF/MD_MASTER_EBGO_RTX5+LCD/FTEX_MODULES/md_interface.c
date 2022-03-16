/**
  ******************************************************************************
  * @file    vc_config.h
  * @author  Jorge Polo, FTEX
  * @brief   Module that has functions for getting (and setting) the 
  *          values of the motor drive parameters
	******************************************************************************
*/

#include "md_interface.h"

void MD_Init(MD_Handle_t *p_Handle)
{
	DRT_Init(&p_Handle->DeratingHandler);
}

void MD_setStatus(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDStateMachine.bMState = (MC_State_t)(value & 0xFF);
}

void MD_setBusVoltage(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.bus_voltage_mes = value;
}

void MD_setIqMeas(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.iq_mes = value;
}

void MD_setIdMeas(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.id_mes = value;
}

void MD_setIqRef(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.iq_ref = value;
}

void MD_setIdRef(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.id_ref = value;
}

void MD_setMotorTemp(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.temp_motor = value;
}

void MD_setHeatsinkTemp(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.temp_hs = value;
}

void MD_setSpeedMeas(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.speed_mes = value;
}

void MD_setFlags(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDStateMachine.hMFaultNow = value & 0xFF;
	p_Handle->MDStateMachine.hMFaultOccurred = (value >> 8) & 0xFF;
}

void MD_setTorqueKp(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.torque_kp = value;
}

void MD_setTorqueKi(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.torque_ki = value;
}

void MD_setFluxKp(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.flux_kp = value;
}

void MD_setFluxKi(MD_Handle_t *p_Handle, int32_t value)
{
	p_Handle->MDMeas.flux_ki = value;
}

