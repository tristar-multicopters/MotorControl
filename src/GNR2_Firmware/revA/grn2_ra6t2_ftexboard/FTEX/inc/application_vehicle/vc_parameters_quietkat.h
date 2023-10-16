/**
  * @file    vc_parameters_quietkat.h
  * @brief   This file contains the parameters needed in order to configure the motor of quietkat bike.
  *
  */

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_QUIETKAT_H
#define __VC_PARAMETERS_QUIETKAT_H

#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "uCAL_UART.h"

/******************* SCREEN SELECTION  *******************************/
#define SCREEN_PROTOCOL    UART_APT       
                                                            // UART_DISABLE,
                                                            // UART_APT,
                                                            // UART_KD718,
                                                            // UART_CLOUD_5S,                                              
                                                            // UART_LOG_HS 
                                              
/***************** THROTTLE SELECTION  ******************************/

#include "throttle_sensors/wuxing_130x.h"

/***************** THROTTLE FEEL PARAMETERS  ******************************/
#define THROTTLE_OFFSET_THROTTLE2TORQUE        1            // Offset for throttle to torque linear transformation
#define THROTTLE_OFFSET_THROTTLE2SPEED         1            // Offset for throttle to speed linear transformation 

#define THROTTLE_DETECTION_THRESHOLD        1000            // Throttle is considered pressed once it passed this threshold

/***************** PEDAL SENSOR SELECTION  ******************************/

#include "pedal_sensors/dashi_p14l.h"

/***************** PEDDLE ASSIST SYSTEM PARAMETERS  ******************************/
#define PAS_MAX_TORQUE                      STARTING_TORQUE // Maximum motor torque to apply using pedal assist
#define PAS_MAX_LEVEL                         5             // Maximum PAS Level given by the screen   
#define PAS_TORQUE_GAIN                     100             // Torque sensor PAS Gain in % (100% is normal, < 100% is a reduction, > 100% is an increase in power) 
#define PAS_MAX_TORQUE_RATIO                100             // Maximum PAS Torque feed ration in 100%    
#define PAS_ALGORITHM                       CadenceSensorUse/* TorqueSensorUse  = 0, Torque sensor use define 
                                                               CadenceSensorUse = 1, Cadence sensor use define */
    
#define PAS_C_0_POWER_PERCENT               0               // PAS 0 has a ratio of   0%
#define PAS_C_1_POWER_PERCENT               35              // PAS 1 has a ratio of  35%
#define PAS_C_2_POWER_PERCENT               45              // PAS 2 has a ratio of  45%
#define PAS_C_3_POWER_PERCENT               60              // PAS 3 has a ratio of  60%
#define PAS_C_4_POWER_PERCENT               80              // PAS 4 has a ratio of  80%
#define PAS_C_5_POWER_PERCENT               100             // PAS 5 has a ratio of 100%
#define PAS_WALK_POWER_PERCENT              70              // PAS walk has a ratio of 70%

#define PAS_MIN_PEDAL_COUNT_SAFE            3               // Number of pulse per pedal turn do we neeed after initial detection to push power
#define PAS_WALKMODE_OVER_THROTTLE          true            // If set to true walk mode has higher priority than throttle

/************** WHEEL SPEED SENSOR SELECTION (MOTOR SIGNALS) *****************************/

#include "speed_sensors/bafang_g062_750w.h"

/***************** MOTOR SELECTOR PARAMETERS  ******************************/

#define MOTOR_SELECTOR_ENABLE               true            // True if active motor can be changed using 3 way switch

/***************** POWER ENABLE PARAMETERS  ******************************/

#define POWER_ENABLE_ENABLE                 true            // True if power enable input is used to prevent powertrain start

/***************** POWERTRAIN MANAGEMENT ******************************/

#define POWERTRAIN_USE_MOTOR1               true            // True if motor1 is used
#define POWERTRAIN_USE_MOTOR2               true            // True if motor2 is used
#define POWERTRAIN_DEFAULT_MAIN_MOTOR       M1              // Default main motor, can be M1 or M2
#define POWERTRAIN_DEFAULT_MODE             DUAL_MOTOR      // Default powertrain mode, can be SINGLE_MOTOR or DUAL_MOTOR
#define POWERTRAIN_M2_TORQUE_INVERSION      false           // If true, M2 torque is inverted compared to M1
#define POWERTRAIN_START_THROTTLE_THRESHOLD 1000            // Throttle value to start powertrain
#define POWERTRAIN_STOP_THROTTLE_THRESHOLD  100             // Throttle value to stop powertrain
#define POWERTRAIN_STOP_SPEED_THRESHOLD     0               // Speed value to stop powertrain
#define POWERTRAIN_DISABLE_THROTTLE_PAS_0   true            // If set the tru throttle is disabled when pas level is 0

#define DYNAMIC_SPEED_LIMITATION            false           // Indicates if the the top speed change be changed dynamically or is fixed to the default value
                                                                
#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT 200             /*  Number of task ticks to wait after a fault occurs to 
                                                                attempt a powertrain restart (OC, SF and SU faults)   */

#define POWERTRAIN_MAX_MOTOR_TORQUE         STARTING_TORQUE // Maximum motor torque to apply with powertrain management

/***************** BIKE LIGHT SETTINGS  ******************************/

#define POWERTRAIN_HEADLIGHT_LOCKED         false           // Parameter that decides if the user can change the state of the headlight      
#define POWERTRAIN_HEADLIGHT_DEFAULT        false           // Parameter that sets the default headlight state when the bike is powered on

#define POWERTRAIN_TAILLIGHT_LOCKED         false           // Parameter that decide sif the user can change the state of the tail light 
#define POWERTRAIN_TAILLIGHT_DEFAULT        false           // Parameter that sets the default tail light state when the bike is powered on

/******************************** BATTERY SELECTION ******************************/

#include "batteries/pathfinder_battery.h"

#endif                                            

