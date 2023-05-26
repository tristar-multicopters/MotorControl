/**
  * @file    gnr_parameters.h
  * @brief   This file contains general parameters used by ganrunner firmware
*/

#define VEHICLE_DEFAULT               0
#define VEHICLE_GRIZZLY               1
#define VEHICLE_E_CELLS               2
#define VEHICLE_R48_750W              3
#define VEHICLE_WHEEL                 4
#define VEHICLE_TSUGAWA               5
#define VEHICLE_NIDEC                 6
#define VEHICLE_QUIETKAT              7
#define VEHICLE_RBS_MB                8
#define VEHICLE_URBAN                 9
#define VEHICLE_VELEC_CITI_500W       10 
#define VEHICLE_A2_350W               11

/*______________________________________________________*/
/* Change parameter below to quickly configure firmware */

//support slave was added to enable master + iot to
//suport or not the slave gnr. 
//if master == 1 and iot == 1 and dual motor is present
//support slave must be set to 1.
//if iot == 0 support slave state don't care.
//This means that firmware will ignore SUPPORT_SLAVE_ON_IOT 
//and can be configured as before.
//Some examples:
//
//GNR_IOT 0, GNR_MASTER 1 -> configure gnr to be a master
//that can communicate with a slave if he is present.
//on this setup SUPPORT_SLAVE_ON_IOT value doesn't make any
//difference. This is the way our system was working.
//
//GNR_IOT 0, GNR_MASTER 0 -> configure gnr to be a slave.
//on this setup SUPPORT_SLAVE_ON_IOT value doesn't make any
//difference. This is the way our system was working.
//
//GNR_IOT 1, GNR_MASTER 1, SUPPORT_SLAVE_ON_IOT 1-> configure gnr 
//to be a master that can communicate with iot module and
//a slave if they are present.
//
//GNR_IOT 1, GNR_MASTER 1, SUPPORT_SLAVE_ON_IOT 0-> configure gnr 
//to be a master that can communicate with only with a IOT module
//if iot is present.
//
//to know how to use the three define bellow, please 
//check the description above.
#define GNR_IOT                       0                   /* If IOT, controller manages canbus communication with IOT Module and discard vehicule communication.  */
#define GNR_MASTER                    1                  /* If master, controller manages canbus communication with slaves and vehicle control layer.  */
#define SUPPORT_SLAVE_ON_IOT          0                   /* if slaver, enable slaver to send, by TPDO, informationa about him self to the master
                                                             and be controlled by the master.*/
#define GNR2_MASTER_NODE_ID           0x01                /* Node-ID of ganrunner master */
#define IOT_NODE_ID                   0x02                /* Node-ID of the IOT module(configured in the IOT firmware)*/ 
#define GNR2_SLAVE_NODE_ID            0x03                /* Node-ID of ganrunner slave */

#define SWD_CONTROL_ENABLE            0                   /* Enable controlling motor directly from debugging interface with vehicle control layer */
#define DEBUGMODE_MOTOR_CONTROL       0                   /* Disable vehicle control and communications to debug only motor control layer */
#define ENABLE_MC_DAC_DEBUGGING       0                   /* Update DAC outputs during FOC interrupt */
#define ENABLE_VC_DAC_DEBUGGING       0                   /* Update DAC outputs during VC medium frequency task */



#define VEHICLE_SELECTION             VEHICLE_QUIETKAT     /* Vehicle selection to adapt motor/vehicle parameters.
                                                           Will be changed in the future for a more flexible way
                                                           of parametrization. */
                                                                                                               
//define responsible to enable some debug features as:
//hardware fault catch 
//#define HFAULTDEBUG

//define responsible to enable canloggertask
//#define CANLOGGERTASK

//define responsible to enable or disable throttle speed control
#define THROTTLE_SPEED_CTRL

//define to allow screne to change power
//#define SCREENPOWERCONTROL 


/*
  Set BYPASS_POSITION_SENSOR to 1 and BYPASS_CURRENT_CONTROL to 1 for simple open loop voltage output.
  Set BYPASS_POSITION_SENSOR to 1 and BYPASS_CURRENT_CONTROL to 0 for simple current control without position sensor.
  Set BYPASS_POSITION_SENSOR to 0 and BYPASS_CURRENT_CONTROL to 0 for normal operation.
*/
#define BYPASS_POSITION_SENSOR        0     /* Hall sensor is bypassed; instead angle is increasing at constant speed. */
#define BYPASS_CURRENT_CONTROL        0    /* Current control is bypassed, thus outputing a constant voltage */
#define DYNAMIC_CURRENT_CONTROL_PID   1
/*______________________________________________________*/
/*Used to enable High speed Log Configuration*/

//definition used to enable log function

#define USE_HSLOG 0 // Define to 1 to enable HS logger

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
