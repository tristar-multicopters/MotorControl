/**
  ******************************************************************************
  * @file    vc_config.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module declares global structures used by other vehicule control modules
  *
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VC_CONFIG_H
#define __VC_CONFIG_H

#include "board_hardware.h"

#include "md_comm.h"
#include "throttle.h"
#include "vc_state_machine.h"
#include "torque_distribution.h"
#include "brake.h"
#include "canbus_management.h"


#define USE_MOTOR1
#define USE_MOTOR2


typedef struct {
	int32_t wheelDiameter;
	int32_t maxSpeed;
	int32_t gearRatio;
} VehiculeParameters_t;


typedef struct {
	CAN_Handle_t* pCANbusManager;
	Brake_Handle_t* pBrake;
	RCM_Handle_t* pRegularConvertionManager;
	Throttle_Handle_t * pThrottle;
	TD_Handle_t * pTorqueDistributor;
	VCSTM_Handle_t * pSTM;
	MD_Comm_Handle_t * pMDComm;
	VehiculeParameters_t * pVehiculeParam;
	bool isWaitingForReply;
} VC_Handle_t;


extern VC_Handle_t VControl;
extern MD_Handle_t MotorDrive1;
extern MD_Handle_t MotorDrive2;


#endif /* __VC_CONFIG_H */

