/**
  * @file    gnr_parameters.h
  * @brief   This file contains general parameters used by ganrunner firmware
*/

#define VEHICLE_DEFAULT               0
#define VEHICLE_EBGO                  1
#define VEHICLE_GRIZZLY               2
#define VEHICLE_E_CELLS               3
#define VEHICLE_APOLLO                4
#define VEHICLE_VELEC                 5
#define VEHICLE_WHEEL                 6
#define VEHICLE_TSUGAWA               7

/*______________________________________________________*/
/* Change parameter below to quickly configure firmware */
#define GNR_IOT                       0                   /* If IOT, controller manages canbus communication with IOT Module and discard vehicule communication.  */
#define GNR_MASTER                    1                   /* If master, controller manages canbus communication with slaves and vehicle control layer.  */
#define GNR2_MASTER_NODE_ID           0x01                /* Node-ID of ganrunner master */
#define GNR2_SLAVE_NODE_ID            0x02                /* Node-ID of ganrunner slave */

#define SWD_CONTROL_ENABLE            0            /* Enable controlling motor directly from debugging interface with vehicle control layer */
#define DEBUGMODE_MOTOR_CONTROL       0                   /* Disable vehicle control and communications to debug only motor control layer */
#define ENABLE_MC_DAC_DEBUGGING       0                   /* Update DAC outputs during FOC interrupt */
#define ENABLE_VC_DAC_DEBUGGING       0                   /* Update DAC outputs during VC medium frequency task */
#define VEHICLE_SELECTION             VEHICLE_TSUGAWA     /* Vehicle selection to adapt motor/vehicle parameters.
                                                         Will be changed in the future for a more flexible way
                                                         of parametrization. */
                                                         
//define responsible to enable some debug features as:
//hardware fault catch 
#define HFAULTDEBUG 0 

//define responsible to enable canloggertask
#define CANLOGGERTASK 0 
/*
  Set BYPASS_POSITION_SENSOR to 1 and BYPASS_CURRENT_CONTROL to 1 for simple open loop voltage output.
  Set BYPASS_POSITION_SENSOR to 1 and BYPASS_CURRENT_CONTROL to 0 for simple current control without position sensor.
  Set BYPASS_POSITION_SENSOR to 0 and BYPASS_CURRENT_CONTROL to 0 for normal operation.
*/
#define BYPASS_POSITION_SENSOR        0     /* Hall sensor is bypassed; instead angle is increasing at constant speed. */
#define BYPASS_CURRENT_CONTROL        0     /* Current control is bypassed, thus outputing a constant voltage */
#define DYNAMIC_CURRENT_CONTROL_PID   1
/*______________________________________________________*/
/*High speed Log Configuration*/

//definition used to enable log function
//LogHS_StartOneShot(&LogHS_handle);
#define HSLOG_ZEROSPEED_LOG 0

//definition used to enable log function
//LogHS_LogMotorVals(&LogHS_handle);
#define LOGMOTORVALS 0

//definition used to enable log function
//LogHS_LogMotorValsVarRes(&LogHS_handle);
#define LOGMOTORVALSRES 0

////definition used to enable log function
//LogHS_StopLog(&LogHS_handle);
#define HSLOG_BUTTON_LOG 0 
 

