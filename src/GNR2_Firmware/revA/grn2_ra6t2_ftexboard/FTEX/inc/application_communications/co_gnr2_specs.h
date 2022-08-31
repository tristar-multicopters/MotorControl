/**
* @file   co_gnr2_specs.h
* @author Jorge Polo
* @brief  Module for defining the CANOpen specs of GNR2
*
* This module defines the Object dictionary of the GNR2
* and also all the node caracteristics according to
* CANOpen protocol
*/

#ifndef CO_GNR2_SPECS_H_
#define CO_GNR2_SPECS_H_

#ifdef __cplusplus               /* for compatibility with C++ environments  */
extern "C" {
#endif

// ================================== INCLUDES ================================== //

#include "co_core.h"

// ================================== PUBLIC DEFINES ================================== //

/* Application specific CANopen registers */
#define CO_OD_REG_MOTOR_SPEED              0x2600
#define CO_OD_REG_BUS_VOLTAGE              0x2601
#define CO_OD_REG_MOTOR_TEMP               0x2602
#define CO_OD_REG_HEATSINK_TEMP            0x2603
#define CO_OD_REG_MOTOR_STATE              0x2604
#define CO_OD_REG_MOTOR_OCC_FAULTS         0x2605
#define CO_OD_REG_MOTOR_CUR_FAULTS         0x2606
#define CO_OD_REG_MOTOR_TORQUE_REF         0x2607
#define CO_OD_REG_MOTOR_START              0x2608
#define CO_OD_REG_FAULT_ACK                0x2609


/* Specify the EMCY-IDs for the application */
enum EMCY_CODES {
	APP_NO_ERROR = 0,
	APP_ERR_MOTOR_OVER_TEMPERATURE,
	APP_ERR_MOTOR_STARTUP_FAULT,
	APP_ERR_CURRENT_FAULT,
	APP_ERR_HALL_SENSOR_FAULT,
	APP_ERR_WHEEL_SPEED_SENSOR_FAULT,
	APP_ERR_CONTROLLER_OVER_VOLTAGE,
	APP_ERR_CONTROLLER_UNDER_VOLTAGE,
	APP_ERR_CONTROLLER_OVER_TEMPERATURE,
	APP_ERR_THROTTLE_FAULT,
	APP_ERR_PAS_FAULT,

	APP_ERR_ID_NUM            /* number of EMCY error codes in application */
};

// ================================== PUBLIC SYMBOLS ================================== //

extern struct CO_NODE_SPEC_T GnR2ModuleSpec;

#ifdef __cplusplus               /* for compatibility with C++ environments  */
}
#endif

#endif /* CO_GNR2_SPECS_H_ */