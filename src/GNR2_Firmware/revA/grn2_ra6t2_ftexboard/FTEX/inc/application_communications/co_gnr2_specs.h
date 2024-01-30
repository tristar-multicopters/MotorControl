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

//Number of entries to each OD configuration

#define MASTER_IOT_SIZE       11
#define MASTER_IOT_SLAVE_SIZE 15
#define MASTER_SLAVE_SIZE     14
#define SLAVE_MASTER_SIZE     14

//Mandatory Object Entries address

//OD ID for Device type
#define CO_OD_REG_DEVICE_TYPE       0x1000

//OD ID to hold the errors.
#define CO_OD_REG_ERROR             0x1001

//OD ID to sync message
#define CO_OD_REG_SYNC_MESSAGE      0x1005

//OD ID to sync period
#define CO_OD_REG_SYNC_PERIOD       0x1006

//OD ID to the Emergency message
#define CO_OD_REG_EMCY_MSG          0x1014

//OD ID to Producer Heartbeat Time
#define CO_OD_REG_HB_TIME           0x1017

//OD ID to Identity Object.
#define CO_OD_IDENTITY_OBJECT       0x1018
//this object has 5 subindex 
// 0 -> Highest sub-index supported.
// 1 -> Vendor ID.
// 2 -> Product code
// 3 -> Revision number
// 4 -> Serial Number

//OD ID to Communication Object SDO Server
#define CO_OD_SDO_SERVER            0x1200
//this object has 3 subindex 
// 0 -> inform Highest sub-index supported.
// 1 -> SDO Server Request COBID
// 2 -> SDO Server Response COBID

//OD ID to SDO Client - a device to request information
#define CO_OD_SDO_CLIENT_01            0x1280
//this object has 4 subindex 
// 0 -> Highest sub-index supported.
// 1 -> COB-ID Client to Server
// 2 -> COB-ID Server to Client   
// 3 -> Node-ID of the SDO server

//OD ID to SDO Client - a device to request information
#define CO_OD_SDO_CLIENT_02            0x1281
//this object has 4 subindex 
// 0 -> Highest sub-index supported.
// 1 -> COB-ID Client to Server
// 2 -> COB-ID Server to Client   
// 3 -> Node-ID of the SDO server

//OD ID to RPDO communication parameter
#define CO_OD_RPOD                     0x1400
//this object has 4 subindex 
// 0 -> Highest sub-index supported.
// 1 -> COB-ID used by RPDO
// 2 -> Transmission type  
// 5 -> Event timer

//OD ID to RPDO mapping parameter
#define CO_OD_RPOD_MAPPING             0x1600
//this object can have until 8 subindex 
// 0 -> Number of mapped application objects in PDO
// 1 -> Application object 1
// 2 -> Application object 2 
// 3 -> Application object 3
// 4 -> Application object 4
// .....
// 8 -> Application object 8

//OD ID to TPDO communication parameter
#define CO_OD_TPDO_COMMUNICATION       0x1800
//this object can have until 8 subindex 
// 0 -> Highest sub-index supported
// 1 -> COB-ID used by TPDO
// 2 -> Transmission type 
// 5 -> Event timer
// 6 -> SYNC start value

//OD ID to RPDO mapping parameter
#define CO_OD_TPOD_MAPPING             0x1A00
//this object can have until 8 subindex 
// 0 -> Number of mapped application objects in PDO
// 1 -> Application object 1
// 2 -> Application object 2 
// 3 -> Application object 3
// 4 -> Application object 4
// .....
// 8 -> Application object 8

//OD ID to communn entries, used by all configuration.
#define CO_OD_COMMUM_ENTRIES           0x5000


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
#define CO_OD_REG_PAS_MAX_TORQUE_RATIO        0x2016

//OD ID to setup some PAS parameters on startup condition.
//The define(s) used by this parameter is:
//
//Startup config speed. Speed at which startup configuration becomes active.
//SUBINDEX 0: hStartupOffsetMTSpeedKMH = PTS_OFFSET_STARTUP_SPEED_KMH
//
//Startup threshold of the torque sensor (if present). 
//Threshold in % of pressure needed to push power (when under the startup config speed)
//SUBINDEX 1: hOffsetMTStartup = PTS_OFFSET_PTS2TORQUE_STARTUP.
//
//Minimum number of pulses to detect PAS cadence.
//SUBINDEX 2: sParameters.bPASMinPulseCount = PAS_MIN_PEDAL_PULSE_COUNT.
//
//Time window for PAS cadence detection on start condition.
//SUBINDEX 3: pPSS->wPedalSpeedSens_Windows = CADENCE_DETECTION_WINDOWS_MS.

#define CO_OD_REG_PAS_DETECTION_STARTUP        0x2017

//OD ID for configure torque Sensor Multiplier(GAIN)
//PAS ramp multiplication coefficient for a better user feeling
//The define used by this parameter is:
//
//Torque Gain to PAS 0
//SUBINDEX 0: sParameters.bTorqueGain[0] = PAS_0_TORQUE_GAIN
//
//Torque Gain to PAS 1
//SUBINDEX 1: sParameters.bTorqueGain[1] = PAS_1_TORQUE_GAIN
//
//Torque Gain to PAS 2
//SUBINDEX 2: sParameters.bTorqueGain[2] = PAS_2_TORQUE_GAIN
//
//
//
//Torque Gain to PAS 9
//SUBINDEX 9: sParameters.bTorqueGain[9] = PAS_9_TORQUE_GAIN
#define CO_OD_REG_TORQUE_SENSOR_MULTIPLIER     0x2018

//OD ID for configure torque Maximum speed
//The define used by this parameter is:
//
#define CO_OD_REG_PAS_MIN_TORQUE             0x2019

//OD ID for configure the speed for each Pas Level.
//Maximum Speed for PAS Level x in Km/h
//The define(s) used by this parameter is:
//.sParameters.PASMaxSpeed
//Max Cadence on Speed PAS 0
//SUBINDEX 0: sParameters.PASMaxSpeed[0] = PAS_LEVEL_SPEED_0
//
//Max Cadence on Speed PAS 1
//SUBINDEX 1: sParameters.PASMaxSpeed[1] = PAS_LEVEL_SPEED_1
//
////Max Cadence on Speed PAS 2
//SUBINDEX 2: sParameters.PASMaxSpeed[2] = PAS_LEVEL_SPEED_2
//
//.......
//
////Max Cadence on Speed PAS 9
//SUBINDEX 9: sParameters.PASMaxSpeed[9] = PAS_LEVEL_SPEED_9
//NOTE: this OD ID will have 10 subindexs.
#define CO_OD_REG_PAS_LEVEL_SPEED         0x201A

//OD ID for configure Troque level Power.
//The define(s) used by this parameter is:
//
//NOTE: this OD ID will have 10 subindexs.
#define CO_OD_REG_PAS_MAX_TORQUE           0x201B

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

// OD ID to set the wheel with 3 subindexs
//variable associated with CO_OD_REG_WHEELS_DIAMETER 0 
//wheel diameter in inches
//variable associated with CO_OD_REG_WHEELS_DIAMETER 1 
//Wheel Pulse Per Rotation  (not implemented but subinde xis alreayd picked)     
//variable associated with CO_OD_REG_WHEELS_DIAMETER 2 
//Wheel Diameter Default  

#define CO_OD_REG_WHEELS_DIAMETER              0x201F
    


#define CO_OD_REG_VEHICLE_FRONT_LIGHT          0x2021
//variable associated with CO_OD_REG_VEHICLE_FRONT_LIGHT 0 
// OD ID to operate the front light (0 Off, 1 On) 	

//variable associated with CO_OD_REG_VEHICLE_FRONT_LIGHT 1
// Get or set vehicle front light's default state 0 = Off by default. This means the light will be off when the bike turns on,
//                                                 1= On  by default: turn on when the controller turns on (and off when the controller turns off)

#define CO_OD_REG_VEHICLE_REAR_LIGHT           0x2022
//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT 0 
// OD ID to operate the rear light (0 Off, 1 On) 	

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT 1
// Get or set vehicle rear light's default state 0 = Off by default. This means the light will be off when the bike turns on,
//                                                 1= On  by default: turn on when the controller turns on (and off when the controller turns off)

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT 2
// Get or set the lights brake behavior.
//0 = no special behavior on brake 1 = blink on brake

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT 3 PLACEHOLDER
//Get or set the rear light blinking period. This changes the speed at which the rear light blinks if set to blink, 

//variable associated with CO_OD_REG_VEHICLE_REAR_LIGHT 4 PLACEHOLDER
//Get or set the rear light duty cycle when blinking. The % represents the % of time the light should be on when the light is blinking.

// OD ID used on dual motor to detect if the GNR master/slave
//communication still on.
//Doesn't matter the value write on this address, important
//is the response from CANOPEN layer to informing the other
//side(master or slave) received the sdo download command.
#define CO_OD_REG_MASTER_SLAVE_PRESENT         0x2023


// OD ID used to configure the charateristics of a pas sensor
/*
COD_OD_REG_PAS_SENSOR 0 (Placeholder currently not implemented) 
Current pedaling cadence from the PAS sensor, in RPM

COD_OD_REG_PAS_SENSOR 1 (Placeholder currently not implemented) 
Current torque detected on the pedals (if torque sensor present), expressed in %

COD_OD_REG_PAS_SENSOR 2 (Placeholder currently not implemented) 
Current torque detected on the pedals (if torque sensor present), expressed in watts

COD_OD_REG_PAS_SENSOR 3
Configure the number of magnets per rotation of the PAS cadence sensor

COD_OD_REG_PAS_SENSOR 4
PAS torque sensor min value (ie. Offset), between 0-65535  (0 means 0V, 65535 means 5V)

COD_OD_REG_PAS_SENSOR 5
PAS torque sensor max value, between 0-65535  (0 means 0V, 65535 means 5V)
*/
#define COD_OD_REG_PAS_SENSOR                  0x2024

// OD ID to set the screen protocol 
#define CO_OD_CONFIG_SCREEN_PROTOCOL           0x2025


// Available                                   0x2026 


// Available                                   0x2027 


// Available                                   0x2028


// Available                                   0x2029


// Available                                   0x202A


// OD ID to set the throttle adc offset
#define CO_OD_CONFIG_THROTTLE_ADC_OFFSET       0x202B

// OD ID to set the throttle adc max
#define CO_OD_CONFIG_THROTTLE_ADC_MAX          0x202C

// OD ID to configure speed value to each filter band.
//The define(s) used by this parameter is:

//Speed value used to decide what pair of filter bands will be used.
//SUBINDEX 0: hFilterSpeed[0] = PTS_SPEED_FILTER_1
//Speed value used to decide what pair of filter bands will be used.
//SUBINDEX 1: hFilterSpeed[1] = PTS_SPEED_FILTER_2
#define CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER        0x202D

// OD ID to configure filter bands for each speed
//The define(s) used by this parameter is:

//Torque filter band 1 when speed is under the speed torque band.
//SUBINDEX 0: hLowPassFilterBW1[0] = PTS_FILTER_BW1_1
//
//Torque filter band 2 when speed is under the speed torque band.
//SUBINDEX 1: hLowPassFilterBW2[0] = PTS_FILTER_BW2_1
//
//Torque filter band 1 when speed is above the speed torque band.
//SUBINDEX 2: hLowPassFilterBW1[1] = PTS_FILTER_BW1_2
//
//Torque filter band 2 when speed is above the speed torque band.
//SUBINDEX 3: hLowPassFilterBW2[1] = PTS_FILTER_BW2_2
//Torque filter band 1 when speed is above the speed torque band.
//SUBINDEX 4: hLowPassFilterBW1[2] = PTS_FILTER_BW1_3
//
//Torque filter band 2 when speed is above the speed torque band.
//SUBINDEX 5: hLowPassFilterBW2[2] = PTS_FILTER_BW2_3
#define CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED    0x202E

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


// !!! IMPORTANT !!! // !!! IMPORTANT !!! // !!! IMPORTANT !!! //
//
// If you add a new object in the dictionnairy make sure the following document is up-to-date !
// https://tristarmulticopters.atlassian.net/wiki/spaces/ERR/pages/520126468/Evionics+Power+CAN+object+dictionary
//
// !!! IMPORTANT !!! // !!! IMPORTANT !!! // !!! IMPORTANT !!! //


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
  @brief Function used to config the OD as master/iot or slave.
         Must be called before initialise CANOPEN Node.
         Must be called only once.
  @param deviceState true to setup as master, false to setup as slave.
  @retval none
 */
void CO_SelecOdSetup(bool deviceFunction);


#ifdef __cplusplus               /* for compatibility with C++ environments  */
}
#endif

#endif /* CO_GNR2_SPECS_H_ */
