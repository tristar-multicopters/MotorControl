/**
  * @file    vc_parameters_r48_750w.h
  * @brief   This file contains the parameters needed in order to configure the motor of VEHICLE_R48_750W bike.
  *
  */

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_R48_750W_H
#define __VC_PARAMETERS_R48_750W_H

/******************* MOTOR SELECTION  *******************************/
#define MOTOR_SELECTION    MOTOR_AKM_128SX_750W


/******************* SCREEN SELECTION  *******************************/
#define SCREEN_PROTOCOL    UART_CLOUD_5S       
                                                            // UART_DISABLE,
                                                            // UART_APT,
                                                            // UART_KD718,
                                                            // UART_CLOUD_5S,                                              
                                                            // UART_LOG_HS 

/******************* SPEED LIMIT PARAMETERS  *******************************/

#define ENABLE_SPEED_LIMIT                     true         // Enable or disable speed limit (default speed limit is 32km/hr)
#define VEHICLE_TOP_SPEED_KMH              32    
#define THROTTLE_TOP_SPEED                     VEHICLE_TOP_SPEED_KMH

/***************** THROTTLE SELECTION  ******************************/

#include "throttle_sensors/vc_velec_r48_throttle.h"

/***************** THROTTLE FEEL PARAMETERS  ******************************/
#define THROTTLE_OFFSET_THROTTLE2TORQUE        1            // Offset for throttle to torque linear transformation  */
#define THROTTLE_OFFSET_THROTTLE2SPEED         1            // Offset for throttle to speed linear transformation 

#define THROTTLE_DETECTION_THRESHOLD        1000            // Throttle is considered pressed once it passed this threshold

/***************** PEDAL SENSOR SELECTION  ******************************/

#include "pedal_sensors/duratorq_autorq.h"

/***************** TORQUE SENSOR FEEL PARAMETERS  ******************************/
#define PTS_OFFSET_PTS2TORQUE_STARTUP       80              // Offset for pedal torque sensor to torque linear transformation during the startup in %
#define PTS_OFFSET_STARTUP_SPEED_KMH         3              // Speed under which the Startup pedal torque sensor offset is used in km/h
#define PTS_OFFSET_PTS2TORQUE               10              // Offset for pedal torque sensor to torque linear transformation in %
#define PTS_OFFSET_PTS2TORQUE_SAFETY        40              // Offset for pedal torque sensor to torque linear transformation that is considered safe in %

/***************** TORQUE SENSOR FILTERING  ******************************/
#define PTS_FILTER_BW1_1                      40              // BW coefficient for pedal torque sensor avereging for speed 1
#define PTS_FILTER_BW2_1                      90              // BW coefficient for pedal torque sensor avereging for speed 1
#define PTS_FILTER_BW1_2                      40              // BW coefficient for pedal torque sensor avereging for speed 2
#define PTS_FILTER_BW2_2                      90              // BW coefficient for pedal torque sensor avereging for speed 2
#define PTS_FILTER_BW1_3                      40              // BW coefficient for pedal torque sensor avereging for speed 3
#define PTS_FILTER_BW2_3                      90              // BW coefficient for pedal torque sensor avereging for speed 3

/***************** PEDAL ASSIST SYSTEM PARAMETERS  ******************************/
#define PAS_MAX_LEVEL                       5               // Maximum PAS Level given by the screen

#define PAS_0_TORQUE_GAIN                   0               // Torque sensor PAS Gain in % (100% is normal, < 100% is a reduction, > 100% is an increase in power)
#define PAS_1_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 1
#define PAS_2_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 2                
#define PAS_3_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 3
#define PAS_4_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 4
#define PAS_5_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 5
#define PAS_6_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 6
#define PAS_7_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 7 
#define PAS_8_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 8 
#define PAS_9_TORQUE_GAIN                   125             // Torque sensor PAS Gain in % on PAS 9 

#define PAS_MAX_TORQUE_RATIO                100             // Maximum PAS Torque feed ration in 100%

// Global PAS Params
#define RUNTIME_PAS_SPEED_THRESHOLD                 5.0f    // Speed threshold to enable startup PAS power in km/h
#define STARTUP_PAS_SPEED_THRESHOLD                 1.0f    // Speed threshold to enable runtime PAS power in km/h
#define TORQUE_STARTUP_VALUE_THRESHOLD    (uint16_t)80      // Torque value (%) that needs to be provided to have a startup detection
#define STARTUP_PULSE_NUMBER              (uint32_t)2       // Number of pulses that needs to be detected to trigger startup detection
#define STARTUP_TIME_WINDOW               (uint16_t)1000    // Time window (ms) in which the startup pulse number is counted
#define RUNTIME_PULSE_NUMBER              (uint32_t)1       // Number of pulses that needs to be detected to trigger runtime detection
#define RUNTIME_TIME_WINDOW               (uint16_t)400     // Time window (ms) in which the runtime pulse number is counted

// Flag used to detect PAS with cadence AND torque.
// 0: Cadence OR Torque
// 1: Cadence AND Torque
#define CADENCE_AND_OR_TORQUE    0

// Select the ramp type from the enum in file _ramp.h_
#define PAS_RAMP_SELECTION  DYNAMIC_DECELERATION_RAMP

// Dynamic Deceleration Ramp Params
#define DYNAMIC_DECEL_RAMP_START              0.0f          // Min speed(km/h) where the dynamic deceleration ramp starts
#define DYNAMIC_DECEL_RAMP_END                32.0f         // Max speed(km/h) where the dynamic deceleration ramp ends
#define DYNAMIC_DECEL_RAMP_POWER_MIN_SPEED    0.5f          // Dynamic deceleration ramp value(in % of MAX power) at max speed(km/h)
#define DYNAMIC_DECEL_RAMP_POWER_MAX_SPEED    100.0f        // Dynamic deceleration ramp value(in % of MAX power) at min speed(km/h)

// High Speed Power Limiting Ramp Params
#define HIGH_SPEED_POWER_LIMITING_RAMP_START              0.0f          // Min speed(km/h) where the dynamic deceleration ramp starts
#define HIGH_SPEED_POWER_LIMITING_RAMP_END                32.0f         // Max speed(km/h) where the dynamic deceleration ramp ends
#define HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MIN_SPEED    50.0f         // Dynamic deceleration ramp value(in % of MAX power) at max speed(km/h)
#define HIGH_SPEED_POWER_LIMITING_RAMP_POWER_MAX_SPEED    100.0f        // Dynamic deceleration ramp value(in % of MAX power) at min speed(km/h)

// Torque Scaling by Pedaling RPM Options

// Flag used to activate the torque scaling option
// true : option will be activated
// false : option will be decativated/bypassed
#define TORQUE_SCALING_ACTIVATED    true

#define MIN_RPM_SCALING    (uint16_t)2          // Minimum pedaling RPM where the torque gain scaling starts
#define MAX_RPM_SCALING    (uint16_t)50         // Maximum pedaling RPM where the torque gain scaling stops
#define GAIN_AT_MIN_RPM              100.0f     // Torque scaling gain (in %) applied at min RPM
#define GAIN_AT_MAX_RPM              300.0f     // Torque scaling gain (in %) applied at max RPM


//Used to chose the pas detection mode on startup, torque or/and cadence,
//torque only or cadence only.
#define PAS_DETECTIONSTARTUP_ALGORITHM      CadenceSensorUse /*noSensorUse = 0,
                                                            TorqueSensorUse,    // Torque sensor use define
                                                            CadenceSensorUse,   // Cadence sensor use define
                                                            HybridAndSensorUse, // Torque AND Cadence sensor use define
                                                            HybridOrSensorUse,  // Torque OR Cadence sensor use define*/
//Used to chose the pas detection mode on running, torque or/and cadence,
//torque only or cadence only.
#define PAS_DETECTIONRUNNING_ALGORITHM      CadenceSensorUse /*noSensorUse = 0,
                                                            TorqueSensorUse,    // Torque sensor use define
                                                            CadenceSensorUse,   // Cadence sensor use define
                                                            HybridAndSensorUse, // Torque AND Cadence sensor use define
                                                            HybridOrSensorUse,  // Torque OR Cadence sensor use define*/
                                                               
#define PAS_0_MIN_TORQUE_PERCENT                0
#define PAS_1_MIN_TORQUE_PERCENT               	5
#define PAS_2_MIN_TORQUE_PERCENT               10
#define PAS_3_MIN_TORQUE_PERCENT               15
#define PAS_4_MIN_TORQUE_PERCENT               22
#define PAS_5_MIN_TORQUE_PERCENT               30

#define PAS_0_MAX_TORQUE_PERCENT                0
#define PAS_1_MAX_TORQUE_PERCENT               60
#define PAS_2_MAX_TORQUE_PERCENT               67
#define PAS_3_MAX_TORQUE_PERCENT               80
#define PAS_4_MAX_TORQUE_PERCENT               88
#define PAS_5_MAX_TORQUE_PERCENT              100 

#define PAS_WALK_POWER_PERCENT                 70              // PAS walk has a ratio of 70%

#define PEDALSPEEDSENSOR_MIN_PULSE_STARTUP              5    // Mini Number of pulse, inside a specific time, to the detect PAS on cadence
//cadence detection windows on ms when starting to use the pedal.
#define PEDALSPEEDSENSOR_DETECTION_WINDOWS_STARTUP_MS 1000           

#define PEDALSPEEDSENSOR_MIN_PULSE_RUNNING               1    // Mini Number of pulse, inside a specific time, to the detect PAS on cadence when bike is running    
//cadence detection windows on ms when running. 
#define PEDALSPEEDSENSOR_DETECTION_WINDOWS_RUNNING_MS  150

#define PAS_WALKMODE_OVER_THROTTLE          true            // If set to true walk mode has higher priority than throttle

/***************** MOTOR SELECTOR PARAMETERS  ******************************/

#define MOTOR_SELECTOR_ENABLE               false           // True if active motor can be changed using 3 way switch
  
/***************** POWER ENABLE PARAMETERS  ******************************/

#define POWER_ENABLE_ENABLE                 true            // True if power enable input is used to prevent powertrain start

/***************** WHEEL SPEED SENSOR PARAMETERS  ******************************/

#define WWS_USE_MOTOR_NBR_PER_ROTATION      true            // True if wheel speed sensor from the motor is used
#define EXTERNAL_WSS_NBR_PER_ROTATION       1               // Number of magnets on external wheel speed sensor

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

#define DYNAMIC_SPEED_LIMITATION            false            // Indicates if the the top speed change be changed dynamically or is fixed to the default value

#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT 200             /*  Number of task ticks to wait after a fault occurs to
                                                                attempt a powertrain restart (OC, SF and SU faults)   */

/***************** BIKE LIGHT SETTINGS  ******************************/
    
#define POWERTRAIN_HEADLIGHT_DEFAULT        false           // Parameter that sets the default headlight state when the bike is powered on

#define POWERTRAIN_TAILLIGHT_DEFAULT        true            // Parameter that sets the default tail light state when the bike is powered on
    
/******************************** BATTERY SELECTION ******************************/

#include "batteries/velec_750w_battery.h"
                                            
#endif                                            


