/**
  * @file    vc_parameters_velec_a2.h
  * @brief   This file contains the parameters needed in order to configure the motor of velec A2 bike.
  *
  */

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_VELEC_A2_H
#define __VC_PARAMETERS_VELEC_A2_H

/******************* MOTOR SELECTION  *******************************/
#define MOTOR_SELECTION    MOTOR_AKM_128SX_350W


/******************* SCREEN SELECTION  *******************************/
#define SCREEN_PROTOCOL    UART_APT       
                                                            // UART_DISABLE,
                                                            // UART_APT,
                                                            // UART_KD718,
                                                            // UART_CLOUD_5S,
                                                            // UART_LOG_HS 

/******************* SPEED LIMIT PARAMETERS  *******************************/

#define ENABLE_SPEED_LIMIT                  false           // Enable or disable speed limit (default speed limit is 32km/hr)
#define VEHICLE_TOP_SPEED_KMH               32    
#define THROTTLE_TOP_SPEED                  VEHICLE_TOP_SPEED_KMH

/***************** THROTTLE SELECTION  ******************************/

#include "throttle_sensors/vc_velec_throttle.h"

/***************** THROTTLE FEEL PARAMETERS  ******************************/
#define THROTTLE_OFFSET_THROTTLE2TORQUE     4000            // Offset for throttle to torque linear transformation
#define THROTTLE_OFFSET_THROTTLE2SPEED      400             // Offset for throttle to speed linear transformation 

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

/***************** PEDAL ASSIST SYSTEM PARAMETERS  ******************************/
#define PAS_MAX_LEVEL                       5               // Maximum PAS Level given by the screen
#define PAS_TORQUE_GAIN                     100             // Torque sensor PAS Gain in % (100% is normal, < 100% is a reduction, > 100% is an increase in power)
#define PAS_MAX_TORQUE_RATIO                100             // Maximum PAS Torque feed ration in 100%

// Global PAS Params
#define RUNTIME_PAS_SPEED_THRESHOLD                 5.0f    // Speed threshold to enable startup PAS power in km/h 
#define STARTUP_PAS_SPEED_THRESHOLD                 1.0f    // Speed threshold to enable runtime PAS power in km/h
#define TORQUE_STARTUP_VALUE_THRESHOLD    (uint16_t)10      // Torque value (%) that needs to be provided to have a startup detection
#define STARTUP_PULSE_NUMBER              (uint32_t)5       // Number of pulses that needs to be detected to trigger startup detection
#define STARTUP_TIME_WINDOW               (uint16_t)1000    // Time window (ms) in which the startup pulse number is counted
#define RUNTIME_PULSE_NUMBER              (uint32_t)1       // Number of pulses that needs to be detected to trigger runtime detection
#define RUNTIME_TIME_WINDOW               (uint16_t)150     // Time window (ms) in which the runtime pulse number is counted

// Flag used to detect PAS with cadence AND torque.
// 0: Cadence OR Torque
// 1: Cadence AND Torque
#define CADENCE_AND_OR_TORQUE    0

// Select the ramp type from the enum in file _ramp.h_
#define PAS_RAMP_SELECTION  HIGH_SPEED_POWER_LIMITING_RAMP

// Dynamic Deceleration Ramp Params
#define DYNAMIC_DECEL_RAMP_START              0.0f          // Min speed(km/h) where the dynamic deceleration ramp starts
#define DYNAMIC_DECEL_RAMP_END                32.0f         // Max speed(km/h) where the dynamic deceleration ramp ends
#define DYNAMIC_DECEL_RAMP_POWER_MIN_SPEED    50.0f         // Dynamic deceleration ramp value(in % of MAX power) at max speed(km/h)
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
#define PAS_DETECTIONSTARTUP_ALGORITHM      HybridAndSensorUse /*noSensorUse = 0,
                                                            TorqueSensorUse,    // Torque sensor use define
                                                            CadenceSensorUse,   // Cadence sensor use define
                                                            HybridAndSensorUse, // Torque AND Cadence sensor use define
                                                            HybridOrSensorUse,  // Torque OR Cadence sensor use define*/
//Used to chose the pas detection mode on running, torque or/and cadence,
//torque only or cadence only.
#define PAS_DETECTIONRUNNING_ALGORITHM      HybridOrSensorUse /*noSensorUse = 0,
                                                            TorqueSensorUse,    // Torque sensor use define
                                                            CadenceSensorUse,   // Cadence sensor use define
                                                            HybridAndSensorUse, // Torque AND Cadence sensor use define
                                                            HybridOrSensorUse,  // Torque OR Cadence sensor use define*/
    
#define PAS_WALK_POWER_PERCENT              70              // PAS walk has a ratio of 70%

#define PEDALSPEEDSENSOR_MIN_PULSE_STARTUP        6    // Mini Number of pulse, inside a specific time, to the detect PAS on cadence
#define PEDALSPEEDSENSOR_MIN_PULSE_RUNNING        6    // Mini Number of pulse, inside a specific time, to the detect PAS on cadence when bike is running
#define PAS_WALKMODE_OVER_THROTTLE          true            // If set to true walk mode has higher priority than throttle

/************** WHEEL SPEED SENSOR SELECTION (MOTOR SIGNALS) *****************************/

#define DEFAULT_WHEEL_DIAMETER_OVERRIDE     26              // Overrides the default wheel diameter value 

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

#define DYNAMIC_SPEED_LIMITATION            false           // Indicates if the the top speed change be changed dynamically or is fixed to the default value

#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT 200             /* Number of task ticks to wait after a fault occurs to
                                                               attempt a powertrain restart (OC, SF and SU faults)   */

/***************** BIKE LIGHT SETTINGS  ******************************/

#define POWERTRAIN_HEADLIGHT_DEFAULT        false           // Parameter that sets the default headlight state when the bike is powered on

#define POWERTRAIN_TAILLIGHT_DEFAULT        true            // Parameter that sets the default tail light state when the bike is powered on
    
/******************************** BATTERY SELECTION ******************************/

#include "batteries/velec_350w_battery.h"
                                            
#endif                                            

