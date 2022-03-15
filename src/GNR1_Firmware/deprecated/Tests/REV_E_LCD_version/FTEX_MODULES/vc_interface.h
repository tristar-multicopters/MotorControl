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
#include "euart_manager.h"

#define MD_PARAMS_SIZE   10
#define VC_PARAMS_SIZE   5
#define TH_PARAMS_SIZE   4

/******************** IDs Table for STRG management module ********************/
typedef enum
{														 /*Index*/
  VC_PARAM_ID_WHEELD,					/* 0 */
  VC_PARAM_ID_MAXSPEED,				/* 1 */
  VC_PARAM_ID_GEARRATIO,			/* 2 */
	VC_PARAM_ID_DUALMOTOR,			/* 3 */
	VC_PARAM_ID_MAINMOTOR,			/* 4 */
} VC_Params_ID_t;

typedef enum
{
	VC_PARAM_ID_THROTTLE_TYPE,	/* 0 */
	VC_PARAM_ID_THROTTLE_M,			/* 1 */
	VC_PARAM_ID_THROTTLE_F,			/* 2 */	
	VC_PARAM_ID_THROTTLE_OFFSET,/* 3 */
} TH_params_ID_t;
	
typedef enum
{
  MD_PARAM_ID_TORQUE_KP,			/* 0 */
  MD_PARAM_ID_TORQUE_KI,			/* 1 */
  MD_PARAM_ID_FLUX_KP,				/* 2 */
	MD_PARAM_ID_FLUX_KI,				/* 3 */
	// MD_PARAM_ID_ANGLE,				
	 MD_PARAM_ID_FF_C1,					/* 4 */
	// MD_PARAM_ID_FF_C2,
	// MD_PARAM_ID_FF_C3,
	// MD_PARAM_ID_FF_KP,
	// MD_PARAM_ID_FW_KI,
	MD_PARAM_ID_MAXVOLT,				/* 5 */
	MD_PARAM_ID_MINVOLT,				/* 6 */
	MD_PARAM_ID_MAXPHASECURRENT,/* 7 */
	MD_PARAM_ID_MAXTEMP,				/* 8 */
	MD_PARAM_ID_POLEPAIRS				/* 9 */
} MD_Params_ID_t;

/*****************************************************************************/
typedef struct {
	int32_t params[VC_PARAMS_SIZE];  // Vehicle  params
} VehiculeParameters_t;

typedef struct
{
	int32_t params[MD_PARAMS_SIZE];
}MotorParameters_t;

typedef struct{
	int32_t hTempThreshold;
	int32_t hSlope;
}DRT_strg_values_t;

// Structure for save all vehicle parameters in the flash memory
typedef struct {
	VehiculeParameters_t vc_params;						// Vehicle params
	MotorParameters_t md_params[NB_OF_MOTOR]; // Motor params
	int32_t th_params[TH_PARAMS_SIZE]; 				// Throttle params
	DRT_strg_values_t drt_values[NB_OF_MOTOR];// Derate params 
	eUART_protocol_t euart_type;							  // Type of device connected to the external UART
	int8_t isOverwritting;
} VParams_for_STRG_t;

typedef struct {
	Brake_Handle_t* pBrake;
	Throttle_Handle_t * pThrottle;
	TD_Handle_t * pTorqueDistributor;
	VCSTM_Handle_t * pSTM;
	MD_Comm_Handle_t * pMDComm;
  VehiculeParameters_t * pVehiculeParam;
	MotorParameters_t * pMotorParams[NB_OF_MOTOR];
	PAS_Handle_t * pPedalAssist;
	eUART_protocol_t euart_type;
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

/* Function to get torque Kp gain */
int16_t VC_getMotorTorqueKp(VC_Handle_t* pHandle, uint8_t motorselection);

/* Function to get torque Ki gain */
int16_t VC_getMotorTorqueKi(VC_Handle_t* pHandle, uint8_t motorselection);

/* Function to get flux Kp gain */
int16_t VC_getMotorFluxKp(VC_Handle_t* pHandle, uint8_t motorselection);

/* Function to get flux Ki gain */
int16_t VC_getMotorFluxKi(VC_Handle_t* pHandle, uint8_t motorselection);

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
