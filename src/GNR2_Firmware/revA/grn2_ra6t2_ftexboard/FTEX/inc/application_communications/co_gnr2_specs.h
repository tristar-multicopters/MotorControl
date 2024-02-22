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

//OD ID used for the Device type
#define CO_OD_REG_DEVICE_TYPE                  0x1000

//OD ID used to hold the errors.
#define CO_OD_REG_ERROR                        0x1001

//OD ID used to sync message
#define CO_OD_REG_SYNC_MESSAGE                 0x1005

//OD ID used to sync period
#define CO_OD_REG_SYNC_PERIOD                  0x1006

//OD ID used for the Emergency message
#define CO_OD_REG_EMCY_MSG                     0x1014

//OD ID used to Produce the Heartbeat Time
#define CO_OD_REG_HB_TIME                      0x1017

//OD ID used to Identity the Object.
//this object has 5 subindex 
//
// 0 -> Highest sub-index supported.
// 1 -> Vendor ID.
// 2 -> Product code
// 3 -> Revision number
// 4 -> Serial Number
#define CO_OD_IDENTITY_OBJECT                  0x1018

//OD ID used for the Communication Object SDO Server
//this object has 3 subindex 
//
// 0 -> inform Highest sub-index supported.
// 1 -> SDO Server Request COBID
// 2 -> SDO Server Response COBID
#define CO_OD_SDO_SERVER                       0x1200

//OD ID used for the SDO Client - a device to request information
//this object has 4 subindex
//
// 0 -> Highest sub-index supported.
// 1 -> COB-ID Client to Server
// 2 -> COB-ID Server to Client   
// 3 -> Node-ID of the SDO server
#define CO_OD_SDO_CLIENT_01                    0x1280

//OD ID used for the SDO Client - a device to request information
//this object has 4 subindex 
//
// 0 -> Highest sub-index supported.
// 1 -> COB-ID Client to Server
// 2 -> COB-ID Server to Client   
// 3 -> Node-ID of the SDO server
#define CO_OD_SDO_CLIENT_02                    0x1281

//OD ID used for RPDO communication parameters
//this object has 4 subindex 
//
// 0 -> Highest sub-index supported.
// 1 -> COB-ID used by RPDO
// 2 -> Transmission type  
// 5 -> Event timer
#define CO_OD_RPOD                             0x1400

//OD ID used for RPDO mapping parameters
//this object can have up to 8 subindex 
//
// 0 -> Number of mapped application objects in PDO
// 1 -> Application object 1
// 2 -> Application object 2 
// 3 -> Application object 3
// 4 -> Application object 4
// .....
// 8 -> Application object 8
#define CO_OD_RPOD_MAPPING                     0x1600

//OD ID used for TPDO communication parameter
//this object can have up to 8 subindex 
//
// 0 -> Highest sub-index supported
// 1 -> COB-ID used by TPDO
// 2 -> Transmission type 
// 5 -> Event timer
// 6 -> SYNC start value
#define CO_OD_TPDO_COMMUNICATION               0x1800

//OD ID used for RPDO mapping parameter
//this object can have up to 8 subindex
//
// 0 -> Number of mapped application objects in PDO
// 1 -> Application object 1
// 2 -> Application object 2 
// 3 -> Application object 3
// 4 -> Application object 4
// .....
// 8 -> Application object 8
#define CO_OD_TPOD_MAPPING                     0x1A00

//OD ID used for communn entries, used by all configuration.
#define CO_OD_COMMUM_ENTRIES                   0x5000


/* FTEX Application specific CANopen registers  */


#define CO_OD_REG_SPEED_MEASURE                0x2000 /**< OD ID for speed measure           >*/

//OD ID used to hold the vehicle power/torque values
//this object has 2 subindex
//
// 0 -> power measure in watts
// 1 -> torque measure in Ncm
#define CO_OD_REG_POWER_MEASURE                0x2001 

#define CO_OD_REG_SOC                          0x2002 /**< OD ID for state of charge measure >*/
#define CO_OD_REG_PAS_LEVEL                    0x2003 /**< OD ID for PAS level               >*/
#define CO_OD_REG_MAX_PAS                      0x2004 /**< OD ID for maximum PAS level       >*/
#define CO_OD_REG_MAX_POWER                    0x2005 /**< OD ID for maximum power           >*/
#define CO_OD_REG_ERR_STATE                    0x2006 /**< OD ID for error state             >*/
#define CO_OD_REG_SERIAL_NB                    0x2007 /**< OD ID for Serial number           >*/
#define CO_OD_REG_FW_VERSION                   0x2008 /**< OD ID for firmware version        >*/

/* Application specific CANopen registers for Gnr */
#define CO_OD_REG_MOTOR_SPEED                  0x2009
#define CO_OD_REG_BUS_VOLTAGE                  0x200A
#define CO_OD_REG_MOTOR_TEMP                   0x200B
#define CO_OD_REG_HEATSINK_TEMP                0x200C
#define CO_OD_REG_MOTOR_STATE                  0x200D
#define CO_OD_REG_MOTOR_OCC_FAULTS             0x200E
#define CO_OD_REG_MOTOR_CUR_FAULTS             0x200F
#define CO_OD_REG_MOTOR_TORQUE_REF             0x2010
#define CO_OD_REG_MOTOR_START                  0x2011
#define CO_OD_REG_FAULT_ACK                    0x2012

/*used by Power off sequency.*********/

//OD ID used to let the slave to know if the device is going to turn off or not.
//IOT module must implement the same address on their side and 
//read this register periodocally(less than 1 second)
//to know if the master will turn off or not.
#define CO_OD_REG_DEVICE_TURNNING_OFF          0x2013

/* User Data configuration for the Gnr */

//OD ID used to inform if user data was updated or is being updated.
//this object has 2 subindex 
//
// 0 -> KEY_USER_DATA_CONFIG_BEING_UPDATED   0xD5A3 : indicate memory being upadted
// 1 -> KEY_USER_DATA_CONFIG_UPDATED         0xC2E5 : indicate memory was updated
#define CO_OD_REG_KEY_USER_DATA_CONFIG         0x2014

/********************************************************/

//OD ID used to configure PAS algorithm. 
//Chose what algorithm will be used by the system.
//PAS algorithm can be:
//TorqueSensorUse = 0,     Torque sensor use define 
//CadenceSensorUse = 1,    Cadence sensor use define 
//HybridSensorUse = 2,     Hybride sensor use define
#define CO_OD_REG_PAS_ALGORITHM                0x2015   

//OD ID used to configure the PAS max power.
//Maximum PAS Torque feed ratio in 100%.
//The define used by this parameter is:
//PAS_MAX_TORQUE_RATIO                  100  
#define CO_OD_REG_PAS_MAX_TORQUE_RATIO         0x2016

//OD ID used to setup some PAS parameters on startup condition.
//this object has 5 subindex 
//
// 0 -> hStartupOffsetMTSpeedKMH = PTS_OFFSET_STARTUP_SPEED_KMH
//     Startup config speed. Speed at which startup configuration becomes active.
//
// 1 -> hOffsetMTStartup = PTS_OFFSET_PTS2TORQUE_STARTUP.
//     Startup threshold of the torque sensor (if present). 
//     Threshold in % of pressure needed to push power (when under the startup config speed)
//
// 2 -> sParameters.hPedalSpeedSens_MinPulseStartup = PEDALSPEEDSENSOR_MIN_PULSE_STARTUP.
//     Minimum number of pulses to detect PAS cadence.
//
// 3 -> pPSS->wPedalSpeedSens_WindowsStartup = PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS.
//     Time window for PAS cadence detection on start condition.
//
// 4 -> paPowertrain->pPAS->bStartupPasAlgorithm = PAS_DETECTIONSTARTUP_ALGORITHM
//     PasAlgorithmStartup for decide what algo will be used to detected PAS on startup.
#define CO_OD_REG_PAS_DETECTION_STARTUP        0x2017

//OD ID used to configure the torque Sensor Multiplier(GAIN)
//PAS ramp multiplication coefficient for a better user feeling
//this object has 10 subindex 
//
// 0 -> sParameters.bTorqueGain[0] = PAS_0_TORQUE_GAIN
//     Torque Gain to PAS 0
//
// 1 -> sParameters.bTorqueGain[1] = PAS_1_TORQUE_GAIN
//     Torque Gain to PAS 1
//
// 2 -> sParameters.bTorqueGain[2] = PAS_2_TORQUE_GAIN
//     Torque Gain to PAS 2
// .....
// 9 -> sParameters.bTorqueGain[9] = PAS_9_TORQUE_GAIN
//     Torque Gain to PAS 9
#define CO_OD_REG_TORQUE_SENSOR_MULTIPLIER     0x2018

//OD ID used to configure the pas minimum torque 
#define CO_OD_REG_PAS_MIN_TORQUE               0x2019

//OD ID used to configure the speed for each Pas Level.
//Maximum Speed for PAS Level x in Km/h
//The define(s) used by this parameter is:
//.sParameters.PASMaxSpeed
// this object has 10 subindex 
//
// Max Cadence on Speed PAS 0
// 0 -> sParameters.PASMaxSpeed[0] = PAS_LEVEL_SPEED_0
//
// Max Cadence on Speed PAS 1
// 1 -> sParameters.PASMaxSpeed[1] = PAS_LEVEL_SPEED_1
//
// Max Cadence on Speed PAS 2
// 2 -> sParameters.PASMaxSpeed[2] = PAS_LEVEL_SPEED_2
//.....
// Max Cadence on Speed PAS 9
// 9 -> sParameters.PASMaxSpeed[9] = PAS_LEVEL_SPEED_9
#define CO_OD_REG_PAS_LEVEL_SPEED              0x201A

//OD ID used to the configure Torque level Power.
//The define(s) used by this parameter is:
//this object has 10 subindex
#define CO_OD_REG_PAS_MAX_TORQUE               0x201B

//OD ID used to configure the  Maximum speed.
//Maximum Bike Speed in Km/h using RPM
//The define(s) used by this parameter is:
//PAS_MAX_KM_SPEED
#define CO_OD_REG_MAX_SPEED                    0x201C

//OD ID used to configure the walk mode speed.
//Maximum bike speed when on the walk mode.
//The define(s) used by this parameter is:
//PAS_LEVEL_SPEED_WALK
#define CO_OD_REG_WALK_MODE_SPEED              0x201D

//OD ID used to configure battery voltages
//The define(s) used by this parameter are:
//BATTERY_FULL_VOLT_X_100
//BATTERY_EMPTY_VOLT_X_100
// this object has 2 subindex
//
// 0 -> Battery full voltage.  Useful for battery SOC calculation in volts x100
// 1 -> Battery empty voltage. Useful for battery SOC calculation in volts x100
#define CO_OD_REG_BATTERY_VOLTAGE              0x201E

//OD ID used to configure wheel parameters
// this object has 3 subindex
//
// 0 -> wheel diameter in inches 
// 1 -> Wheel Pulse Per Rotation  (not implemented but subindex is already picked)     
// 2 -> Wheel Diameter Default in inches 
#define CO_OD_REG_WHEELS_DIAMETER              0x201F

//OD ID used to configure and operate the front light
// this object has 2 subindex
//
// 0 -> operate the front light (0 Off, 1 On) 	
// 1 -> Get or set vehicle front light's default state 
//     0 = Off by default. This means the light will be off when the bike turns on,
//     1 = On  by default: turn on when the controller turns on (and off when the controller turns off)
#define CO_OD_REG_VEHICLE_FRONT_LIGHT          0x2021

//OD ID used to configure and operate the rear light
// this object has 5 subindex
//
// 0 -> operate the rear light (0 Off, 1 On) 	
// 1 -> Get or set vehicle rear light's default state 
//     0 = Off by default. This means the light will be off when the bike turns on,
//     1 = On  by default: turn on when the controller turns on (and off when the controller turns off)
// 2 -> Get or set the lights brake behavior.
//     0 = no special behavior on brake 
//     1 = blink on brake
// 3 -> Get or set the rear light blinking period. 
//     This changes the speed at which the rear light blinks if set to blink, 
// 4 -> Get or set the rear light duty cycle when blinking. 
//     The % represents the % of time the light should be on when the light is blinking.

#define CO_OD_REG_VEHICLE_REAR_LIGHT           0x2022

//OD ID used on dual motor to detect if the GNR master/slave
//communication still on.
//Doesn't matter the value write on this address, important
//is the response from CANOPEN layer to informing the other
//side(master or slave) received the sdo download command.
#define CO_OD_REG_MASTER_SLAVE_PRESENT         0x2023

//OD ID used to configure the charateristics of a pas sensor
// this object has 6 subindex
//
// 0 -> Current pedaling cadence from the PAS sensor, in RPM
//     (Placeholder currently not implemented) 
// 1 -> Current torque detected on the pedals (if torque sensor present), expressed in %
//     (Placeholder currently not implemented) 
// 2 -> Current torque detected on the pedals (if torque sensor present), expressed in watts
//     (Placeholder currently not implemented) 
// 3 -> Configure the number of magnets per rotation of the PAS cadence sensor
// 4 -> PAS torque sensor min value (ie. Offset), between 0-65535  (0 means 0V, 65535 means 3.3V)
// 5 -> PAS torque sensor max value, between 0-65535  (0 means 0V, 65535 means 3.3V)

#define COD_OD_REG_PAS_SENSOR                  0x2024

//OD ID is used to set the screen protocol 
#define CO_OD_CONFIG_SCREEN_PROTOCOL           0x2025

//OD ID is used to configre the battery current limits
// this object has 4 subindex
//
// 0 -> Max peak DC current that the controller can draw from the battery in Amps x 10
// 1 -> Continuous DC current that the controller can draw from the battery Amps X 10
// 2 -> Duration of the peak current, after which the controller starts derating to continuous current in 0.1 Secs
// 3 -> Derating ramp time. This time represents the period of time that the controller uses to derate 
//      from peak to continuous current in 0.1 Secs
#define CO_OD_REG_BATTERY_DC_CURRENT           0x2026

//OD ID is used to control the throttle and to set it's limits
// this object has 6 subindex
//
// 0 -> Throttle ADC Value  [0 - 65535] where 65535 = 5 V
// 1 -> Throttle Get/Set Value in %
// 2 -> Throttle ADC Offset [0 - 65535] where 65535 = 5 V
// 3 -> Throttle ADC Max    [0 - 65535] where 65535 = 5 V
// 4 -> Throttle Block Off  true/false
// 5 -> Throttle Max Speed
#define CO_OD_REG_CONTROLLER_THROTTLE          0x2027

//OD ID used to setup some PAS parameters on running condition.
// this object has 4 subindex
//
// 0 -> hOffsetMTp = PTS_OFFSET_PTS2TORQUE.
//     Startup threshold of the torque sensor (if present). 
//     Threshold in % of pressure needed to push power (when under the startup config speed)
//
// 1 -> sParameters.hPedalSpeedSens_MinPulseStartup = PEDALSPEEDSENSOR_MIN_PULSE_RUNNING.
//     Minimum number of pulses to detect PAS cadence.
//
// 2 -> pPSS->wPedalSpeedSens_WindowsStartup = PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS.
//     Time window for PAS cadence detection on start condition.
//
// 3 -> paPowertrain->pPAS->bRunningPasAlgorithm = PAS_DETECTIONRUNNING_ALGORITHM
//     PasAlgorithmRunning for decide what algo will be used to detected PAS on run time.
#define CO_OD_REG_PAS_DETECTION_RUNNING        0x2028 

// Available                                   0x2029

// Available                                   0x202A

// Available                                   0x202B

// Available                                   0x202C

//OD ID used to configure speed values for each filter band.
// this object has 2 subindex
//
// 0 -> hFilterSpeed[0] = PTS_SPEED_FILTER_1
//     Speed value used to decide what pair of filter bands will be used.
//
// 1 -> hFilterSpeed[1] = PTS_SPEED_FILTER_2
//     Speed value used to decide what pair of filter bands will be used.
#define CO_OD_CONFIG_SPEED_FOR_TORQUE_FILTER   0x202D

//OD ID to configure filter bands for each speed
// this object has 2 subindex
//
// 0 -> hLowPassFilterBW1[0] = PTS_FILTER_BW1_1
//     Torque filter band 1 when speed is under the speed torque band.
//
// 1 -> hLowPassFilterBW2[0] = PTS_FILTER_BW2_1
//     Torque filter band 2 when speed is under the speed torque band.
//
// 2 -> hLowPassFilterBW1[1] = PTS_FILTER_BW1_2
//     Torque filter band 1 when speed is above the speed torque band.
//
// 3 -> hLowPassFilterBW2[1] = PTS_FILTER_BW2_2
//     Torque filter band 2 when speed is above the speed torque band.
//
// 4 -> hLowPassFilterBW1[2] = PTS_FILTER_BW1_3
//     Torque filter band 1 when speed is above the speed torque band.
//
// 5 -> hLowPassFilterBW2[2] = PTS_FILTER_BW2_3
//     Torque filter band 2 when speed is above the speed torque band.
#define CO_OD_CONFIG_TORQUE_FILTER_FOR_SPEED   0x202E

//OD ID will be used to
// receive data and commands during a firmware update.
// the GNR doesn't have enough memory to receive
// all bytes and needs to receive some bytes(more than 4)
// and write them into the external memory.
// this object has 3 subindex 
//
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
