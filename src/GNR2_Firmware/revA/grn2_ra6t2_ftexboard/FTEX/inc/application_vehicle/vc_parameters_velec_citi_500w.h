/**
  * @file    vc_parameters_velec_citi_500w.h
  * @brief   This file contains the parameters needed in order to configure the motor of velec citi 500w bike.
  *
  */

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_VELEC_CITI_500W_H
#define __VC_PARAMETERS_VELEC_CITI_500W_H

#include "drive_parameters.h"

/******************* SCREEN SELECTION  *******************************/
#define SCREEN_PROTOCOL    UART_APT       
                                                            // UART_DISABLE,
                                                            // UART_APT,
                                                            // UART_KD718,
                                                            // UART_CLOUD_5S,                                              
                                                            // UART_LOG_HS 
                                                                             
/***************** THROTTLE SELECTION  ******************************/

#include "throttle_sensors/vc_velec_throttle.h"

/***************** THROTTLE FEEL PARAMETERS  ******************************/
#define THROTTLE_OFFSET_THROTTLE2TORQUE        1            // Offset for throttle to torque linear transformation
#define THROTTLE_OFFSET_THROTTLE2SPEED         1            // Offset for throttle to speed linear transformation 

#define THROTTLE_DETECTION_THRESHOLD        1000            // Throttle is considered pressed once it passed this threshold


/***************** PEDAL SENSOR SELECTION  ******************************/

#include "pedal_sensors/duratorq_autorq.h"

/***************** TORQUE SENSOR FEEL PARAMETERS  ******************************/
#define PTS_OFFSET_PTS2TORQUE_STARTUP       80              // Offset for pedal torque sensor to torque linear transformation during the startup in %
#define PTS_OFFSET_STARTUP_SPEED_KMH        3               // Speed under which the Startup pedal torque sensor offset is used in km/h
#define PTS_OFFSET_PTS2TORQUE               10              // Offset for pedal torque sensor to torque linear transformation in %
#define PTS_OFFSET_PTS2TORQUE_SAFETY        40              // Offset for pedal torque sensor to torque linear transformation that is considered safe in %

/***************** TORQUE SENSOR FILTERING  ******************************/
#define PTS_FILTER_BW1_1                      10              // BW coefficient for pedal torque sensor avereging for speed 1
#define PTS_FILTER_BW2_1                      25              // BW coefficient for pedal torque sensor avereging for speed 1
#define PTS_FILTER_BW1_2                      10              // BW coefficient for pedal torque sensor avereging for speed 2
#define PTS_FILTER_BW2_2                      25              // BW coefficient for pedal torque sensor avereging for speed 2
#define PTS_FILTER_BW1_3                      10              // BW coefficient for pedal torque sensor avereging for speed 3
#define PTS_FILTER_BW2_3                      25              // BW coefficient for pedal torque sensor avereging for speed 3

/***************** PEDDLE ASSIST SYSTEM PARAMETERS  ******************************/
#define PAS_MAX_TORQUE                      NOMINAL_TORQUE  // Maximum motor torque to apply using pedal assist
#define PAS_MAX_LEVEL                       5               // Maximum PAS Level given by the screen
#define PAS_TORQUE_GAIN                     100             // Torque sensor PAS Gain in % (100% is normal, < 100% is a reduction, > 100% is an increase in power)
#define PAS_MAX_TORQUE_RATIO                100             // Maximum PAS Torque feed ration in 100%

//Used to chose what power calculation will be used, 
//torque or cadence.
#define PAS_POWER_ALGORITHM                       TorqueSensorUse /* TorqueSensorUse  = 1, Torque sensor use define 
                                                               CadenceSensorUse = 2, Cadence sensor use define */
//Used to chose the pas detection mode on startup, torque or/and cadence,
//torque only or cadence only.
#define PAS_DETECTIONSTARTUP_ALGORITHM      TorqueSensorUse /*noSensorUse = 0,
                                                            TorqueSensorUse,    // Torque sensor use define
                                                            CadenceSensorUse,   // Cadence sensor use define
                                                            HybridAndSensorUse, // Torque AND Cadence sensor use define
                                                            HybridOrSensorUse,  // Torque OR Cadence sensor use define*/
//Used to chose the pas detection mode on running, torque or/and cadence,
//torque only or cadence only.
#define PAS_DETECTIONRUNNING_ALGORITHM      TorqueSensorUse /*noSensorUse = 0,
                                                            TorqueSensorUse,    // Torque sensor use define
                                                            CadenceSensorUse,   // Cadence sensor use define
                                                            HybridAndSensorUse, // Torque AND Cadence sensor use define
                                                            HybridOrSensorUse,  // Torque OR Cadence sensor use define*/
                                                               
#define PAS_WALK_POWER_PERCENT              70              // PAS walk has a ratio of 70%

#define PEDALSPEEDSENSOR_MIN_PULSE_STARTUP  6               // Mini Number of pulse, inside a specific time, to the detect PAS on cadence
#define PEDALSPEEDSENSOR_MIN_PULSE_RUNNING  6               // Mini Number of pulse, inside a specific time, to the detect PAS on cadence when bike is running
#define PAS_WALKMODE_OVER_THROTTLE          true            // If set to true walk mode has higher priority than throttle

/************** WHEEL SPEED SENSOR SELECTION (MOTOR SIGNALS) *****************************/

#include "speed_sensors/velec_wheel_motor_sensor.h"
    
/***************** MOTOR SELECTOR PARAMETERS  ******************************/

#define MOTOR_SELECTOR_ENABLE               false           // True if active motor can be changed using 3 way switch
  
/***************** POWER ENABLE PARAMETERS  ******************************/

#define POWER_ENABLE_ENABLE                 true            // True if power enable input is used to prevent powertrain start

/***************** POWERTRAIN MANAGEMENT ******************************/

#define POWERTRAIN_USE_MOTOR1               true            // True if motor1 is used
#define POWERTRAIN_USE_MOTOR2               false           // True if motor2 is used
#define POWERTRAIN_DEFAULT_MAIN_MOTOR       M1              // Default main motor, can be M1 or M2
#define POWERTRAIN_DEFAULT_MODE             SINGLE_MOTOR    // Default powertrain mode, can be SINGLE_MOTOR or DUAL_MOTOR
#define POWERTRAIN_M2_TORQUE_INVERSION      false           // If true, M2 torque is inverted compared to M1
#define POWERTRAIN_START_THROTTLE_THRESHOLD 1000            // Throttle value to start powertrain
#define POWERTRAIN_STOP_THROTTLE_THRESHOLD  100             // Throttle value to stop powertrain
#define POWERTRAIN_STOP_SPEED_THRESHOLD     0               // Speed value to stop powertrain
#define POWERTRAIN_DISABLE_THROTTLE_PAS_0   true            // If set the tru throttle is disabled when pas level is 0

#define DYNAMIC_SPEED_LIMITATION            false           // Indicates if the the top speed change be changed dynamically or is fixed to the default value

#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT 200             /* Number of task ticks to wait after a fault occurs to
                                                               attempt a powertrain restart (OC, SF and SU faults)   */

#define POWERTRAIN_MAX_MOTOR_TORQUE         STARTING_TORQUE // Maximum motor torque to apply with powertrain management

/***************** BIKE LIGHT SETTINGS  ******************************/
    
#define POWERTRAIN_HEADLIGHT_DEFAULT        false           // Parameter that sets the default headlight state when the bike is powered on

#define POWERTRAIN_TAILLIGHT_DEFAULT        true            // Parameter that sets the default tail light state when the bike is powered on

/******************************** BATTERY SELECTION ******************************/

#include "batteries/velec_500w_battery.h"
                                            
#endif                                            

