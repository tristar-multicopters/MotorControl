/**
  ******************************************************************************
  * @file    vc_interface.h
	* @author  Jorge A. Polo, FTEX
  * @author  Sami Bouzid, FTEX
  * @brief   This module offers an interface to interact with vehicle properties
	******************************************************************************
	*/

#ifndef __VC_INTERFACE_H
#define __VC_INTERFACE_H

#include "stdlib.h"
#include "mc_defines.h"
#include "md_comm.h"
#include "throttle.h"
#include "vc_state_machine.h"
#include "torque_distribution.h"
#include "brake.h"
#include "pedal_assist.h"

typedef struct {
	int32_t wheelDiameter;
	int32_t maxSpeed;
	int32_t gearRatio;
} VehiculeParameters_t;

typedef struct {
	Brake_Handle_t* pBrake;
	Throttle_Handle_t * pThrottle;
	TD_Handle_t * pTorqueDistributor;
	VCSTM_Handle_t * pSTM;
	MD_Comm_Handle_t * pMDComm;
	VehiculeParameters_t * pVehiculeParam;
	PAS_Handle_t * pPedalAssist;
} VC_Handle_t;


/*** GET INFORMATION FROM VEHICLE ***/

/*	Function to get the current state of the vehicle	*/
VC_State_t VC_getVehicleState(VC_Handle_t* pHandle);

/*	Function to get the current fault (if there's any) of the vehicle*/
uint16_t VC_getVehicleFaultNow(VC_Handle_t* pHandle);

/*	Function to get faults that occured of the vehicle	*/
uint16_t VC_getVehicleFaultOccurred(VC_Handle_t* pHandle);

/*	Function to get the state of vehicle brakes 	*/
bool VC_isBrakeOn(VC_Handle_t* pHandle);

/*	Function to get the average value of the vehicle	*/
uint16_t VC_getThrottle(VC_Handle_t* pHandle);

/*	Function to get the bus voltage value of the battery	*/
int32_t VC_getBattVoltage(VC_Handle_t* pHandle);

/*	Function to get the current state of a motor	*/
MC_State_t VC_getMotorState(VC_Handle_t* pHandle, uint8_t motorselection);

/*	Function to get the current fault (if there's any) of a motor	*/
uint16_t VC_getMotorFaultNow(VC_Handle_t* pHandle, uint8_t motorselection);

/*	Function to get the last fault of a motor	*/
uint16_t VC_getMotorFaultOccurred(VC_Handle_t* pHandle, uint8_t motorselection);

/*	Function to get the measured current values Iq and Id of a motor	*/
qd_t VC_getMotorIqIdMeas(VC_Handle_t* pHandle, uint8_t motorselection);

/*	Function to get the referenced current values Iq and Id of a motor	*/
qd_t VC_getMotorIqIdRef(VC_Handle_t* pHandle, uint8_t motorselection);

/*	Function to get measured speed of a motor	*/
int32_t VC_getMotorSpeedMeas(VC_Handle_t* pHandle, uint8_t motorselection);

/*	Function to get temperature of an inverter heatsink	*/
int32_t VC_getInverterHeatsinkTemp(VC_Handle_t* pHandle, uint8_t motorselection);

/*	Function to get temperature of a motor	*/
int32_t VC_getMotorTemp(VC_Handle_t* pHandle, uint8_t motorselection);


/*** SEND COMMANDS TO VEHICLE ***/

void VC_StartVehicle(VC_Handle_t* pHandle);

void VC_StopVehicle(VC_Handle_t* pHandle);

void VC_SetPASLevel(VC_Handle_t* pHandle);

void VC_SetThrottleValue(VC_Handle_t* pHandle);




#endif
