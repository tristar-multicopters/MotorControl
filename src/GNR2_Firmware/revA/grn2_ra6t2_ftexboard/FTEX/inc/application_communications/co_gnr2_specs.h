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

/************************************************************************************/

//define used to determine the maximum number of bytes used by the variable
//responsible to receive the package of bytes during a firmware update.
#define FIRMWAREUPDATE_MEMORYSIZE 134

//define the sdo client number to be used by the sdo services.
#define SDOCLIENTINDEX_SLAVE        0x00
#define SDOCLIENTINDEX_MASTER       0x00
#define SDOCLIENTINDEX_IOT          0x01

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

/*used by Power off sequency.*********/

//OD ID used to let the slave to know if the device is going to
//tunr off or not.
//IOT module must implement the same address on your side and 
//read this register periodocally(less than 1 second)
//to know if the master will turn off or not.
//
#define CO_OD_REG_DEVICE_TURNNING_OFF          0x2013

/*User Data configuration for Gnr*/

//OD ID for inform if user data was upadted or
//is being upadted.
//There are two possible values that can be written 
//on this address:
//
//KEY_USER_DATA_CONFIG_BEING_UPDATED   0xD5A3 : indicate memory being upadted
//KEY_USER_DATA_CONFIG_UPDATED         0xC2E5 : indicate memory was updated
//
#define CO_OD_REG_KEY_USER_DATA_CONFIG         0x2014

/********************************************************/

/*Throttle/Pedal Assist CANopen registers for Gnr*/
//variables that can be used by canopen, using SDO,
//to configure Throttle/Pedal Assist parameters.

//OD ID for configure PAS algorithm. 
//Chose what algorithm will be used by the system.
//PAS algorithm can be:
//TorqueSensorUse = 0,     Torque sensor use define 
//CadenceSensorUse = 1,    Cadence sensor use define 
//HybridSensorUse = 2,     Hybride sensor use define
#define CO_OD_REG_PAS_ALGORITHM               0x2015   

//OD ID for configure PAS max power.
//Maximum PAS Torque feed ratio in 100%.
//The define used by this parameter is:
//PAS_MAX_TORQUE_RATIO                   99  
#define CO_OD_REG_PAS_MAX_POWER                0x2016

//OD ID for configure torque Minimum Threshold
//The define used by this parameter is:
//PTS_OFFSET_PTS2TORQUE                  20
#define CO_OD_REG_TORQUE_MINIMUM_THRESHOLD     0x2017

//OD ID for configure torque Sensor Multiplier
//PAS ramp multiplication coefficient for a better user feeling
//The define used by this parameter is:
//PAS_LEVEL_COEFF                         1
#define CO_OD_REG_TORQUE_SENSOR_MULTIPLIER     0x2018

//OD ID for configure torque Maximum speed
//The define used by this parameter is:
//
#define CO_OD_REG_TORQUE_MAX_SPEED             0x2019

//OD ID for configure Cadence/hybrid level.
//Maximum Speed for PAS Level x in Km/h
//The define(s) used by this parameter is:
//PAS_LEVEL_SPEED_0
//PAS_LEVEL_SPEED_1
//PAS_LEVEL_SPEED_2
//PAS_LEVEL_SPEED_3
//.....
//NOTE: this OD ID will have 10 subindexs.
#define CO_OD_REG_CADENCE_HYBRID_LEVEL         0x201A

//OD ID for configure Troque level Power.
//The define(s) used by this parameter is:
//
//NOTE: this OD ID will have 10 subindexs.
#define CO_OD_REG_TORQUE_LEVEL_POWER           0x201B

//OD ID for configure Maximum speed.
//Maximum Bike Speed in Km/h using RPM
//The define(s) used by this parameter is:
//PAS_MAX_KM_SPEED
#define CO_OD_REG_MAX_SPEED                    0x201C

//OD ID for configure walk mode speed.
//Maximum bike speed when on the walk mode.
//The define(s) used by this parameter is:
//PAS_LEVEL_SPEED_WALK
#define CO_OD_REG_WALK_MODE_SPEED              0x201D

// OD ID to set the wheel diamater in inches

#define CO_OD_REG_WHEELS_DIAMETER              0x201F



// OD ID to operate the front light (0 Off, 1 On)
#define CO_OD_REG_VEHICLE_FRONT_LIGHT          0x2021


// OD ID to operate the rear light (0 Off, 1 On)
#define CO_OD_REG_VEHICLE_REAR_LIGHT           0x2022

// OD ID used on dual motor to detect if the GNR master/slave
//communication still on.
//Doesn't matter the value write on this address, important
//is the response from CANOPEN layer to informing the other
//side(master or slave) received the sdo download command.
#define CO_OD_REG_MASTER_SLAVE_PRESENT  0x2023

//this OD ID will be used to 
//receive data and commands during a firmware update.
//the GNR doesn't have enough memory to receive
//all bytes and needs to receive some bytes(more than 4)
//and write them into the external memory.
//this object has 3 subindex 
// 0 -> used to receive command from the IOT module to control the DFU process.
// 1 -> used to inform about the ongoing state of the DFU process and report any error.
// 2 -> used to receive the data frame(part of the firware file).
// 3 -> used to inform IOT the last received data frame.
#define CO_OD_REG_FIRMWAREUPDATE_MEMORY        0x3000


/*******************************************************/

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

extern CO_OBJ_DOM bObjFirmwareUpdateDomain;

/**
  @brief Function used to config the OD as master or slave.
  @param deviceState true to setup as master, false to setup as slave.
  @retval none
 */
void CO_Gnr2OdSetupt(bool deviceState);


#ifdef __cplusplus               /* for compatibility with C++ environments  */
}
#endif

#endif /* CO_GNR2_SPECS_H_ */
