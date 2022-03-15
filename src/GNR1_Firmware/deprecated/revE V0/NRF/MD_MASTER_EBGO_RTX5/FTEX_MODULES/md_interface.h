/**
  ******************************************************************************
  * @file    vc_config.h
  * @author  Jorge Polo, FTEX
  * @brief   Header of the module that has functions for getting (and setting) the 
  *          values of the motor drive parameters
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MD_INTERFACE_H
#define __MD_INTERFACE_H

#include "stdlib.h"
#include "mc_defines.h"
#include "derate.h"

#define NB_OF_MOTOR 2

typedef struct {
	int32_t bus_voltage_mes;
	int32_t speed_mes;
	int32_t iq_mes;
	int32_t id_mes;
	int32_t iq_ref;
	int32_t id_ref;
	int32_t temp_hs;
	int32_t temp_motor;
	int32_t torque_kp;
	int32_t torque_ki;
	int32_t flux_kp;
	int32_t flux_ki;
} MD_Measurements_t;

typedef struct
{
	MC_State_t   bMState;       /*!< Variable containing state machine current
                                    state of motor controller */
  uint16_t  hMFaultNow;       /*!< Bit fields variable containing faults
                                    currently present */
  uint16_t  hMFaultOccurred;  /*!< Bit fields variable containing faults
                                    historically occurred since the state
                                    machine has been moved to FAULT_NOW state */
} MDSTM_Handle_t;

typedef struct {
	MD_Measurements_t MDMeas;
	MDSTM_Handle_t MDStateMachine;
	DRT_Handle_t DeratingHandler;
} MD_Handle_t;

//************** PUBLIC FUNCTIONS **********//

/* Initialize motor drive interface */
void MD_Init(MD_Handle_t *p_Handle);

/* Update motor status */
void MD_setStatus(MD_Handle_t *p_Handle, int32_t value);

/* Update Bus voltage */
void MD_setBusVoltage(MD_Handle_t *p_Handle, int32_t value);

/* Update Iq measure */
void MD_setIqMeas(MD_Handle_t *p_Handle, int32_t value);

/* Update Id measure */
void MD_setIdMeas(MD_Handle_t *p_Handle, int32_t value);

/* Update Iq reference */
void MD_setIqRef(MD_Handle_t *p_Handle, int32_t value);

/* Update Id reference */
void MD_setIdRef(MD_Handle_t *p_Handle, int32_t value);

/* Update motor temperature */
void MD_setMotorTemp(MD_Handle_t *p_Handle, int32_t value);

/* Update heatsink temperature */
void MD_setHeatsinkTemp(MD_Handle_t *p_Handle, int32_t value);

/* Update speed measure */
void MD_setSpeedMeas(MD_Handle_t *p_Handle, int32_t value);

/* Update flags */
void MD_setFlags(MD_Handle_t *p_Handle, int32_t value);

/* Update Torque Kp */
void MD_setTorqueKp(MD_Handle_t *p_Handle, int32_t value);

/* Update Torque Ki */
void MD_setTorqueKi(MD_Handle_t *p_Handle, int32_t value);

/* Update Flux Kp */
void MD_setFluxKp(MD_Handle_t *p_Handle, int32_t value);

/* Update Flux Ki */
void MD_setFluxKi(MD_Handle_t *p_Handle, int32_t value);

#endif
