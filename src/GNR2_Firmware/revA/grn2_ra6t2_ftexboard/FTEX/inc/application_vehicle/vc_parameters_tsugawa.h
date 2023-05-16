/**
  * @file    vc_parameters_tsugawa.h
  * @brief   This file contains the parameters needed in order to configure the motor of tsugawa motor.
  *
  */

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_TSUGAWA_H
#define __VC_PARAMETERS_TSUGAWA_H

#include "pmsm_motor_parameters.h"
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

#define THROTTLE_OFFSET_ADC2THROTTLE          12500    // Offset for ADC to throttle linear transformation
#define THROTTLE_MAX_ADC2THROTTLE             53200    // Maximum value reachable by the throttle adc value 

#define THROTTLE_OFFSET_THROTTLE2TORQUE        4000    // Offset for throttle to torque linear transformation

#define THROTTLE_DETECTION_THRESHOLD           1000    // Throttle is considered pressed once it passed this threshold

#define THROTTLE_MAX_SAFE_SPEED_RPM             285    // Max Wheel RPM that is safe when using the motor (aprox 38 km/h)
#define THROTTLE_DEFAULT_MAX_SPEED_RPM          240    // Default top spee din wheel RPM (aprox 32 km/h)

#define THROTTLE_SPEED_DECREASING_RANGE         200    // Number of RPM before the desired speed at which we should start removing power
                                                       // Should be aroud 200 for light bikes and 55 for heavy bikes

/***************** PEDAL TORQUE SENSOR PARAMETERS  ******************************/

#define PTS_FILTER_ALPHA              2.27F    // Butterworth alpha coefficient pedal torque sensor filtering
#define PTS_FILTER_BETA              -0.27F    // Butterworth beta coefficient pedal torque sensor filtering
#define PTS_MAX_PTSVALUE         UINT16_MAX	   // Maximum analog value to reach

#define PTS_OFFSET_ADC2PTS             8500    // Offset for ADC to pedal torque sensor linear transformation

#define PTS_OFFSET_PTS2TORQUE_STARTUP    40    // Offset for pedal torque sensor to torque linear transformation during the startup in %
#define PTS_OFFSET_STARTUP_SPEED         20    // Speed under which the  Startup pedal torque sensor offset is used in wheel rpm
#define PTS_OFFSET_PTS2TORQUE            10    // Offset for pedal torque sensor to torque linear transformation in %

#define PTS_FILTER_BW1                   10    // BW coefficient for pedal torque sensor avereging
#define PTS_FILTER_BW2                   25    // BW coefficient for pedal torque sensor avereging

/************** WHEEL SPEED SENSOR PARAMETERS  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION      1    // Wheel speed sensor cycle number for one wheel rotation
#define WHEEL_SPEED_SLOW_LOOP_COUNT              3    // Wheel speed sensor slow detect counter to get 750ms per function call
#define WHEEL_SPEED_SENSOR_CORRECTION_FACTOR     2    // Wheel speed sensor slow detect correction for a signal after two wheel spin detection
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
#define POWERTRAIN_DEFAULT_CONTROL_TYPE     TORQUE_CTRL    // Default control type, can be TORQUE_CTRL or SPEED_CTRL
#define POWERTRAIN_M2_TORQUE_INVERSION            false    // If true, M2 torque is inverted compared to M1
#define POWERTRAIN_START_THROTTLE_THRESHOLD        1000    // Throttle value to start powertrain
#define POWERTRAIN_STOP_THROTTLE_THRESHOLD          100    // Throttle value to stop powertrain
#define POWERTRAIN_STOP_SPEED_THRESHOLD               0    // Speed value to stop powertrain
#define POWERTRAIN_DISABLE_THROTTLE_PAS_0          true    // If set the tru throttle is disabled when pas level is 0


#define PAS_MAX_TORQUE                             1200    // Maximum motor torque to apply using pedal assist
#define PAS_MAX_SPEED         MAX_APPLICATION_SPEED_RPM    // Maximum motor speed reachable using pedal assist
#define PAS_MAX_KM_SPEED                             28    // Maximum Bike Speed in Km/h using RPM
#define PAS_MAX_LEVEL                                 5    // Maximum PAS Level given by the screen
#define PAS_TORQUE_GAIN                             100    // Torque sensor PAS Gain in % (100% is normal, < 100% is a reduction, > 100% is an increase in power)
#define PAS_MAX_TORQUE_RATIO                         99    // Maximum PAS Torque feed ration in 100%
#define PAS_MAX_SPEED_RATIO                          99    // Maximum PAS Speed feed ration in 100%
#define PAS_ALGORITHM                   HybridSensorUse    /* TorqueSensorUse  = 0, Torque sensor use define 
                                                              CadenceSensorUse = 1, Cadence sensor use define 
                                                              HybridSensorUse  = 2, Hybride sensor use define */

#define PAS_WHEEL_SPEED_SENSOR_PPR                2    // Number of electrical pulses per wheel rotation
#define PAS_MIN_PEDAL_COUNT_SAFE                  0    // Number of pulse per pedal turn do we neeed after initial detection to push power
#define PAS_SLOW_PEDAL_COUNT                      1    // Loop wait counter to update the PAS detection function
#define PAS_WALKMODE_OVER_THROTTLE             true    // If set to true walk mode has higher priority than throttle

#define POWERTRAIN_MOTOR_GEARRATIO                0x000B0005    /* Motor gear ratio, i.e. wheel speed divided by motor speed.
                                                                   Upper half of 32 bits is numerator, 
                                                                   second half is denominator */

#define POWERTRAIN_FAULT_MANAGEMENT_TIMEOUT              200    /* Number of task ticks to wait after a fault occurs to 
                                                                   attempt a powertrain restart (OC, SF and SU faults)   */

#define POWERTRAIN_MAX_MOTOR_TORQUE                       STARTING_TORQUE    // Maximum motor torque to apply with powertrain management
    
#define POWERTRAIN_FOLDBACK_SPEED_END                     MAX_APPLICATION_SPEED_RPM/2    /* Speed value that dual motor startup strategy stops outputting
                                                                                            torque */
#define POWERTRAIN_FOLDBACK_SPEED_RANGE                   MAX_APPLICATION_SPEED_RPM/4    /* Speed interval value between maximum torque and zero torque,
                                                                                            when using dual motor startup strategy. */
#define POWERTRAIN_FOLDBACK_SPEED_INTERVAL                MAX_APPLICATION_SPEED_RPM/15   /* Speed interval value between maximum torque and start torque,
                                                                                            when using single motor startup strategy. */

#define FOLDBACK_SLOW_START_BANDWIDTH                     500    // Fold Back slow start filter bandwidth coefficient for slow ramp to the PAS Control
#define FOLDBACK_SLOW_STOP_BANDWIDTH                      155    // Fold Back slow stop filter bandwidth coefficient for slow ramp to the PAS Control
#define FOLDBACK_TIMEOUT                                  400    // Fold Back Timeout for the slow start ramp

/*********************************Battery Monitoring*******************************/

#define BATTERY_FULL_VOLT    52
#define BATTERY_EMPTY_VOLT   46
                                            
#endif                                            

