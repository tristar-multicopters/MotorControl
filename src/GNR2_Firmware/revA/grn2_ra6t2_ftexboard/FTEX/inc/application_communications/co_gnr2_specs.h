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
#include "gnr_parameters.h"
// ================================== PUBLIC DEFINES ================================== //

/* Application specific CANopen registers for IOT */
#define CO_OD_REG_SPEED_MEASURE     0x2000 /**< OD ID for speed measure           >*/
#define CO_OD_REG_POWER_MEASURE     0x2001 /**< OD ID for power measure           >*/
#define CO_OD_REG_SOC               0x2002 /**< OD ID for state of charge measure >*/
#define CO_OD_REG_PAS_LEVEL         0x2003 /**< OD ID for PAS level               >*/
#define CO_OD_REG_MAX_PAS           0x2004 /**< OD ID for maximum PAS level       >*/
#define CO_OD_REG_MAX_POWER         0x2005 /**< OD ID for maximum power           >*/
#define CO_OD_REG_ERR_STATE         0x2006 /**< OD ID for error state             >*/
#define CO_OD_REG_SERIAL_NB         0x2007 /**< OD ID for Serial number           >*/
#define CO_OD_REG_FW_VERSION        0x2008 /**< OD ID for firmware version        >*/

/* Application specific CANopen registers for Gnr */
#define CO_OD_REG_MOTOR_SPEED       0x2009
#define CO_OD_REG_BUS_VOLTAGE       0x200A
#define CO_OD_REG_MOTOR_TEMP        0x200B
#define CO_OD_REG_HEATSINK_TEMP     0x200C
#define CO_OD_REG_MOTOR_STATE       0x200D
#define CO_OD_REG_MOTOR_OCC_FAULTS  0x200E
#define CO_OD_REG_MOTOR_CUR_FAULTS  0x200F
#define CO_OD_REG_MOTOR_TORQUE_REF  0x2010
#define CO_OD_REG_MOTOR_START       0x2011
#define CO_OD_REG_FAULT_ACK         0x2012

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
