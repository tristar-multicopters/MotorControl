/**
  * @file    vc_parameters_utk_350w.h
  * @brief   This file contains the parameters needed in order to configure the motor of VEHICLE_UTK_W bike.
  *
  */

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_UTK_350W_H
#define __VC_PARAMETERS_UTK_350W_H

#include "drive_parameters.h"

/******************* SCREEN SELECTION  *******************************/
#define SCREEN_PROTOCOL    UART_APT       
                                              // UART_DISABLE,
                                              // UART_APT,
                                              // UART_KD718,    
                                              // UART_LOG_HS 
                                              
/***************** THROTTLE PARAMETERS  ******************************/
#define THROTTLE_FILTER_ALPHA                 2.27F    // Butterworth alpha coefficient for throttle filtering
#define THROTTLE_FILTER_BETA                 -0.27F    // Butterworth beta coefficient for throttle filtering

#define THROTTLE_OFFSET_ADC2THROTTLE          11800    // Offset for ADC to throttle linear transformation
#define THROTTLE_MAX_ADC2THROTTLE             55800    // Maximum value reachable by the throttle adc value

#define THROTTLE_OFFSET_THROTTLE2TORQUE           1    // Offset for throttle to torque linear transformation
#define THROTTLE_OFFSET_THROTTLE2SPEED            1    // Offset for throttle to speed linear transformation 

#define THROTTLE_DETECTION_THRESHOLD           1000    // Throttle is considered pressed once it passed this threshold

/***************** PEDAL TORQUE SENSOR PARAMETERS  ******************************/

#define PTS_FILTER_ALPHA              2.27F    // Butterworth alpha coefficient pedal torque sensor filtering
#define PTS_FILTER_BETA              -0.27F    // Butterworth beta coefficient pedal torque sensor filtering
#define PTS_MAX_PTSVALUE         UINT16_MAX	   // Maximum analog value to reach

#define PTS_OFFSET_ADC2PTS             8500    // Offset for ADC to pedal torque sensor linear transformation

#define PTS_OFFSET_PTS2TORQUE_STARTUP    40    // Offset for pedal torque sensor to torque linear transformation during the startup in %
#define PTS_OFFSET_STARTUP_SPEED_KMH      3    // Speed under which the  Startup pedal torque sensor offset is used in wheel rpm
#define PTS_OFFSET_PTS2TORQUE            10    // Offset for pedal torque sensor to torque linear transformation in %
#define PTS_OFFSET_PTS2TORQUE_SAFETY     40    // Offset for pedal torque sensor to torque linear transformation that is considered safe in %

#define PTS_FILTER_BW1_1                      10              // BW coefficient for pedal torque sensor avereging for speed 1
#define PTS_FILTER_BW2_1                      25              // BW coefficient for pedal torque sensor avereging for speed 1
#define PTS_FILTER_BW1_2                      10              // BW coefficient for pedal torque sensor avereging for speed 2
#define PTS_FILTER_BW2_2                      25              // BW coefficient for pedal torque sensor avereging for speed 2
#define PTS_FILTER_BW1_3                      10              // BW coefficient for pedal torque sensor avereging for speed 3
#define PTS_FILTER_BW2_3                      25              // BW coefficient for pedal torque sensor avereging for speed 3

/************** WHEEL SPEED SENSOR PARAMETERS  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION      6   // Wheel speed sensor cycle number for one wheel rotation
#define WHEEL_SPEED_SLOW_LOOP_COUNT              3    // Wheel speed sensor slow detect counter to get 750ms per function call
#define WHEEL_SPEED_SENSOR_CORRECTION_FACTOR     1   // Wheel speed sensor slow detect correction for a signal after two wheel spin detection
#define WHEEL_SPEED_SLOW_LOOP_DETECT          true    // Wheel speed sensor slow detect counter flag activation
    
/***************** MOTOR SELECTOR PARAMETERS  ******************************/

#define MOTOR_SELECTOR_ENABLE     false    // True if active motor can be changed using 3 way switch
  
/***************** POWER ENABLE PARAMETERS  ******************************/

#define POWER_ENABLE_ENABLE        true    // True if power enable input is used to prevent powertrain start

/***************** POWERTRAIN MANAGEMENT & PAS PARAMETERS  ******************************/

#define POWERTRAIN_USE_MOTOR1                      true    // True if motor1 is used
#define POWERTRAIN_USE_MOTOR2                     false    // True if motor2 is used
#define POWERTRAIN_DEFAULT_MAIN_MOTOR                M1    // Default main motor, can be M1 or M2
#define POWERTRAIN_DEFAULT_MODE            SINGLE_MOTOR    // Default powertrain mode, can be SINGLE_MOTOR or DUAL_MOTOR
#define POWERTRAIN_M2_TORQUE_INVERSION            false    // If true, M2 torque is inverted compared to M1
#define POWERTRAIN_START_THROTTLE_THRESHOLD        1000    // Throttle value to start powertrain
#define POWERTRAIN_STOP_THROTTLE_THRESHOLD          100    // Throttle value to stop powertrain
#define POWERTRAIN_STOP_SPEED_THRESHOLD               0    // Speed value to stop powertrain
#define POWERTRAIN_DISABLE_THROTTLE_PAS_0          true    // If set the tru throttle is disabled when pas level is 0


#define PAS_MAX_TORQUE                   NOMINAL_TORQUE    // Maximum motor torque to apply using pedal assist
#define PAS_MAX_LEVEL                                 5    // Maximum PAS Level given by the screen
#define PAS_TORQUE_GAIN                             100    // Torque sensor PAS Gain in % (100% is normal, < 100% is a reduction, > 100% is an increase in power)
#define PAS_MAX_TORQUE_RATIO                        100    // Maximum PAS Torque feed ration in 100%
#define PAS_ALGORITHM                  CadenceSensorUse    /* TorqueSensorUse  = 0, Torque sensor use define 
                                                              CadenceSensorUse = 1, Cadence sensor use define */
    
#define PAS_0_MAX_TORQUE_PERCENT      0    // PAS 0 has a ratio of   0%
#define PAS_1_MAX_TORQUE_PERCENT     60    // PAS 1 has a ratio of  60% (3/5)
#define PAS_2_MAX_TORQUE_PERCENT     67    // PAS 2 has a ratio of  67% (4/6)
#define PAS_3_MAX_TORQUE_PERCENT     80    // PAS 3 has a ratio of  80% (4/5)
#define PAS_4_MAX_TORQUE_PERCENT     88    // PAS 4 has a ratio of  88% (7/8)
#define PAS_5_MAX_TORQUE_PERCENT    100    // PAS 5 has a ratio of 100%
#define PAS_WALK_POWER_PERCENT  70    // PAS walk has a ratio of 70%

#define PAS_MIN_PEDAL_PULSE_COUNT                 6    // Mini Number of pulse, inside a specific time, to the detect PAS on cadence
#define PAS_SLOW_PEDAL_COUNT                      4    // Loop wait counter to update the PAS detection function
#define PAS_WALKMODE_OVER_THROTTLE             true    // If set to true walk mode has higher priority than throttle

#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT              200    /* Number of task ticks to wait after a fault occurs to 
                                                                   attempt a powertrain restart (OC, SF and SU faults)   */

#define POWERTRAIN_MAX_MOTOR_TORQUE                       STARTING_TORQUE    // Maximum motor torque to apply with powertrain management

/***************** BIKE LIGHT SETTINGS  ******************************/

#define POWERTRAIN_HEADLIGHT_DEFAULT        false    // Parameter that sets the default headlight state when the bike is powered on

#define POWERTRAIN_TAILLIGHT_DEFAULT        false    // Parameter that sets the default tail light state when the bike is powered on
  
/*********************************Battery Monitoring*******************************/

#define BATTERY_FULL_VOLT_X_100  5200
#define BATTERY_EMPTY_VOLT_X_100 4600

#define BATTERY_SOC_LOW_PERCENT   15   // Battery SOC in % for which we set the battery low flag (stops powertrain form pushing power)
#define BATTERY_SOC_OK_PERCENT    25   // Battery SOC in % for which we clear the battery low flag
                                            
#endif                                            

