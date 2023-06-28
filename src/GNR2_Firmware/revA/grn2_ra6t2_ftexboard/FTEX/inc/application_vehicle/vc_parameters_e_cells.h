/**
  * @file    vc_parameters_e_cells.h
  * @brief   This file contains the parameters needed in order to configure the motor of e-cell bike.
  *
  */

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_E_CELLS_H
#define __VC_PARAMETERS_E_CELLS_H

#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"

/******************* SCREEN SELECTION  *******************************/
#define SCREEN_PROTOCOL    UART_APT       
                                                            // UART_DISABLE,
                                                            // UART_APT,
                                                            // UART_KD718,
                                                            // UART_CLOUD_5S,                                              
                                                            // UART_LOG_HS 
                                              
/***************** THROTTLE SELECTION  ******************************/

#include "throttle_sensors/wuxing_20x.h"

/***************** THROTTLE FEEL PARAMETERS  ******************************/

#define THROTTLE_OFFSET_THROTTLE2TORQUE     1               // Offset for throttle to torque linear transformation
#define THROTTLE_OFFSET_THROTTLE2SPEED      400             // Offset for throttle to speed linear transformation 

#define THROTTLE_DETECTION_THRESHOLD        1000            // Throttle is considered pressed once it passed this threshold

#define THROTTLE_MAX_SAFE_SPEED_KMH         40              // Max speed in Km/h that is safe when using the motor
#define THROTTLE_DEFAULT_MAX_SPEED_KMH      32              // Default top speed in km/h

#define THROTTLE_SPEED_DECREASING_RANGE     55              // Number of RPM before the desired speed at which we should start removing power
                                                            // Should be aroud 200 for light bikes and 55 for heavy bikes
/***************** PEDAL SENSOR SELECTION  ******************************/

#include "pedal_sensors/bafang_30nm.h"

/***************** TORQUE SENSOR FEEL PARAMETERS  ******************************/

#define PTS_OFFSET_PTS2TORQUE_STARTUP       70              // Offset for pedal torque sensor to torque linear transformation during the startup in %
#define PTS_OFFSET_STARTUP_SPEED_KMH        3               // Speed under which the Startup pedal torque sensor offset is used in km/h
#define PTS_OFFSET_PTS2TORQUE               10              // Offset for pedal torque sensor to torque linear transformation in %
#define PTS_OFFSET_PTS2TORQUE_SAFETY        40              // Offset for pedal torque sensor to torque linear transformation that is considered safe in %

/***************** TORQUE SENSOR FILTERING  ******************************/

#define PTS_FILTER_BW1                      45              // BW coefficient for pedal torque sensor avereging
#define PTS_FILTER_BW2                      50              // BW coefficient for pedal torque sensor avereging

/***************** PEDDLE ASSIST SYSTEM PARAMETERS  ******************************/

#define PAS_MAX_TORQUE                      STARTING_TORQUE // Maximum motor torque to apply using pedal assist
#define PAS_MAX_SPEED                       MAX_APPLICATION_SPEED_RPM // Maximum motor speed reachable using pedal assist
#define PAS_MAX_KM_SPEED                    90              // Maximum Bike Speed in Km/h using RPM
#define PAS_MAX_LEVEL                       9               // Maximum PAS Level given by the screen    
#define PAS_TORQUE_GAIN                     200             // Torque sensor PAS Gain in % (100% is normal, < 100% is a reduction, > 100% is an increase in power)
#define PAS_MAX_TORQUE_RATIO                100             // Maximum PAS Torque feed ration in 100%   
#define PAS_MAX_SPEED_RATIO                 100             // Maximum PAS Speed feed ration in 100%   
#define PAS_ALGORITHM                       TorqueSensorUse     
                                                            /*  TorqueSensorUse  = 0, Torque sensor use define 
                                                                CadenceSensorUse = 1, Cadence sensor use define 
                                                                HybridSensorUse  = 2, Hybride sensor use define */
#define PAS_CADENCE_USE_SPEED_LIMIT         true            // Decides if we have a speed limit on pas cadence
    
// PAS C (Cadence) power per level setting in %
#define PAS_C_0_POWER_PERCENT               0    
#define PAS_C_1_POWER_PERCENT               60   
#define PAS_C_2_POWER_PERCENT               67   
#define PAS_C_3_POWER_PERCENT               80    
#define PAS_C_4_POWER_PERCENT               88   
#define PAS_C_5_POWER_PERCENT               100    

#define PAS_WALK_POWER_PERCENT              70              // PAS walk has a ratio of 70%

// PAS T (Torque sensor) power per level setting in %
#define PAS_T_0_POWER_PERCENT               0 
#define PAS_T_1_POWER_PERCENT               30
#define PAS_T_2_POWER_PERCENT               39
#define PAS_T_3_POWER_PERCENT               47
#define PAS_T_4_POWER_PERCENT               56
#define PAS_T_5_POWER_PERCENT               65 
#define PAS_T_6_POWER_PERCENT               74
#define PAS_T_7_POWER_PERCENT               83
#define PAS_T_8_POWER_PERCENT               91 
#define PAS_T_9_POWER_PERCENT               100 

#define PAS_MIN_PEDAL_COUNT_SAFE            2               // Number of pulse per pedal turn do we neeed after initial detection to push power
#define PAS_WALKMODE_OVER_THROTTLE          true            // If set to true walk mode has higher priority than throttle

/***************** PEDDLE ASSIST SYSTEM FILTER PARAMETERS  ***************************/

#define FOLDBACK_SLOW_START_BANDWIDTH       500             // Fold Back slow start filter bandwidth coefficient for slow ramp to the PAS Control
#define FOLDBACK_SLOW_STOP_BANDWIDTH        150             // Fold Back slow stop filter bandwidth coefficient for slow ramp to the PAS Control
#define FOLDBACK_TIMEOUT                    400             // Fold Back Timeout for the slow start ramp

/************** WHEEL SPEED SENSOR SELECTION (MOTOR SIGNALS) *****************************/

#include "speed_sensors/bafang_g60_750w.h"

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

#define POWERTRAIN_MOTOR_GEARRATIO          0x000B0005      /*  Motor gear ratio, i.e. wheel speed divided by motor speed.
                                                                Upper half of 32 bits is numerator, 
                                                                second half is denominator */
                                                                
#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT 200             /*  Number of task ticks to wait after a fault occurs to 
                                                                attempt a powertrain restart (OC, SF and SU faults)   */

#define POWERTRAIN_MAX_MOTOR_TORQUE         STARTING_TORQUE // Maximum motor torque to apply with powertrain management

#define POWERTRAIN_FOLDBACK_SPEED_END       MAX_APPLICATION_SPEED_RPM/4    /* Speed value that dual motor startup strategy stops outputting
                                                                              torque */
#define POWERTRAIN_FOLDBACK_SPEED_RANGE     MAX_APPLICATION_SPEED_RPM/8    /* Speed interval value between maximum torque and zero torque,
                                                                              when using dual motor startup strategy. */
#define POWERTRAIN_FOLDBACK_SPEED_INTERVAL  MAX_APPLICATION_SPEED_RPM/15   /* Speed interval value between maximum torque and start torque,
                                                                              when using single motor startup strategy. */

/***************** BIKE LIGHT SETTINGS  ******************************/

#define POWERTRAIN_HEADLIGHT_LOCKED         false           // Parameter that decides if the user can change the state of the headlight      
#define POWERTRAIN_HEADLIGHT_DEFAULT        false           // Parameter that sets the default headlight state when the bike is powered on

#define POWERTRAIN_TAILLIGHT_LOCKED         false           // Parameter that decide sif the user can change the state of the tail light 
#define POWERTRAIN_TAILLIGHT_DEFAULT        false           // Parameter that sets the default tail light state when the bike is powered on

/******************************** BATTERY SELECTION ******************************/

#include "batteries/e_cell_52v_battery.h"

#endif                                            

