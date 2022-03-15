/**
  ******************************************************************************
  * @file    vc_interface.c
	* @author  Jorge A. Polo, FTEX
  * @author  Sami Bouzid, FTEX
  * @brief   This module offers an interface to interact with vehicle properties
	******************************************************************************
	*/
	
#include "vc_interface.h"

/**@brief Function to get the current state of the vehicle.
	 @p p_Handle: Handle of the vehicle
	 @return state of vehicle in a VC_State_t format
*/

VC_State_t VC_getVehicleState(VC_Handle_t* pHandle)
{
	return pHandle->pSTM->bVState;	
}

/**@brief Function to get the current fault (if there's any)	of the vehicle.
	 @p p_Handle: Handle of the vehicle
	 @return fault now
*/

uint16_t VC_getVehicleFaultNow(VC_Handle_t* pHandle)
{
	return pHandle->pSTM->hVFaultNow;	
}

/**@brief Function to get the last fault of the vehicle.
	 @p p_Handle: Handle of the vehicle
	 @return last fault
*/

uint16_t VC_getVehicleFaultOccurred(VC_Handle_t* pHandle)
{
	return pHandle->pSTM->hVFaultOccurred;	
}

/**@brief Function to get the current state of brakes.
	 @p p_Handle: Handle of the vehicle
	 @return returns true if the brakes are on and false if they are released
*/

bool VC_isBrakeOn(VC_Handle_t* pHandle)
{
	return pHandle->pBrake->isPressed;
}

/**@brief Function to get the average throttle value
	 @p p_Handle: Handle of the vehicle
	 @return Average value of throttle acquisition.
*/

uint16_t VC_getThrottle(VC_Handle_t* pHandle)
{
	return pHandle->pThrottle->hAvThrottle;
}

/**@brief Function to get the calculated Iqref
	 @p p_Handle: Handle of the vehicle
	 @return Average value of throttle acquisition.
*/

int32_t VC_getBattVoltage(VC_Handle_t* pHandle)
{
	return pHandle->pMDComm->pMD[M1]->MDMeas.bus_voltage_mes;
}

/**@brief Function to get the current state of a motor.
	 @p p_Handle: Handle of the vehicle
	 @return motor state in MC_State_t format
*/

MC_State_t VC_getMotorState(VC_Handle_t* pHandle, uint8_t motorselection)
{
	return pHandle->pMDComm->pMD[motorselection]->MDStateMachine.bMState;
}

/**@brief Function to get the current fault (if there's any) of a motor.
	 @p p_Handle: Handle of the vehicle
	 @return Current motorselection fault 
*/

uint16_t VC_getMotorFaultNow(VC_Handle_t* pHandle, uint8_t motorselection)
{
	return pHandle->pMDComm->pMD[motorselection]->MDStateMachine.hMFaultNow;
}

/**@brief Function to get the last fault of the motor.
	 @p p_Handle: Handle of the vehicle
	 @return last motorselection fault
*/

uint16_t VC_getMotorFaultOccurred(VC_Handle_t* pHandle, uint8_t motorselection)
{
	return pHandle->pMDComm->pMD[motorselection]->MDStateMachine.hMFaultOccurred;
}

/**@brief Function to get the measured current values (Id and Iq) of a motor.
	 @p p_Handle: Handle of the vehicle
	 @return Measured currents (iq_meas, id_meas)
*/

qd_t VC_getMotorIqIdMeas(VC_Handle_t* pHandle, uint8_t motorselection)
{
	qd_t current_meas;
	current_meas.d = pHandle->pMDComm->pMD[motorselection]->MDMeas.id_mes;
	current_meas.q = pHandle->pMDComm->pMD[motorselection]->MDMeas.iq_mes;
	return current_meas;
}

/**@brief Function to get the reference current values (Id and Iq) of a motor.
	 @p p_Handle: Handle of the vehicle
	 @return Reference currents (iq_ref, id_ref)
*/

qd_t VC_getMotorIqIdRef(VC_Handle_t* pHandle, uint8_t motorselection)
{
	qd_t current_ref;
	current_ref.d = pHandle->pMDComm->pMD[motorselection]->MDMeas.id_ref;
	current_ref.q = pHandle->pMDComm->pMD[motorselection]->MDMeas.iq_ref;
	return current_ref;
}

/**@brief Function to get the torque Kp gain of a motor.
	 @p p_Handle: Handle of the vehicle
	 @return Torque Kp reference and measured speed
*/

int16_t VC_getMotorTorqueKp(VC_Handle_t* pHandle, uint8_t motorselection)
{
	return pHandle->pMDComm->pMD[motorselection]->MDMeas.torque_kp;
}

/**@brief Function to get the torque Ki gain of a motor.
	 @p p_Handle: Handle of the vehicle
	 @return Torque Kp reference and measured speed
*/

int16_t VC_getMotorTorqueKi(VC_Handle_t* pHandle, uint8_t motorselection)
{
	return pHandle->pMDComm->pMD[motorselection]->MDMeas.torque_ki;
}

/**@brief Function to get the torque Ki gain of a motor.
	 @p p_Handle: Handle of the vehicle
	 @return Torque Kp reference and measured speed
*/

int16_t VC_getMotorFluxKp(VC_Handle_t* pHandle, uint8_t motorselection)
{
	return pHandle->pMDComm->pMD[motorselection]->MDMeas.flux_kp;
}

/**@brief Function to get the torque Ki gain of a motor.
	 @p p_Handle: Handle of the vehicle
	 @return Torque Kp reference and measured speed
*/

int16_t VC_getMotorFluxKi(VC_Handle_t* pHandle, uint8_t motorselection)
{
	return pHandle->pMDComm->pMD[motorselection]->MDMeas.flux_ki;
}

/**@brief Function to get measured speed of a motor	
	 @p p_Handle: Handle of the vehicle
	 @return Measured speed
*/

int32_t VC_getMotorSpeedMeas(VC_Handle_t* pHandle, uint8_t motorselection)
{
	int32_t speed_ref;
	speed_ref = pHandle->pMDComm->pMD[motorselection]->MDMeas.speed_mes;
	return speed_ref;
}

/**@brief Function to get temperature of an inverter heatsink	
	 @p p_Handle: Handle of the vehicle
	 @return Inverter heatsink temperature
*/
int32_t VC_getInverterHeatsinkTemp(VC_Handle_t* pHandle, uint8_t motorselection)
{
	int32_t temp;
	temp = pHandle->pMDComm->pMD[motorselection]->MDMeas.temp_hs;
	return temp;
}

/**@brief Function to get temperature of an inverter heatsink	
	 @p p_Handle: Handle of the vehicle
	 @return Inverter heatsink temperature
*/
int32_t VC_getMotorTemp(VC_Handle_t* pHandle, uint8_t motorselection)
{
	int32_t temp;
	temp = pHandle->pMDComm->pMD[motorselection]->MDMeas.temp_motor;
	return temp;
}
