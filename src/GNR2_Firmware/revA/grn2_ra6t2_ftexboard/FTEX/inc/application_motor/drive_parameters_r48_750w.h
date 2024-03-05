/**
  * @file    drive_parameters_R48_750W.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. This file is specific to velec r48 750W motor.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_R48_750W_H
#define __DRIVE_PARAMETERS_R48_750W_H

/******************************** BATTERY SELECTION ******************************/

#include "batteries/velec_750w_battery.h"

/************************** Controller Config  ************************/

#define HARDWARE_SELECTION                 HARDWARE_EP700    // Controller selection to adapt controller parameters

/************************** Motor Config  ************************/
#define MOTOR_GEAR_RATIO                    (float)10.9890      // Motor gear ratio, the value is always X turns of the motor 
#define MOTOR_TYPE                          HUB_DRIVE           // Motor type. Can be HUB_DRIVE or MID_DRIVE
#define MOTOR_TEMP_SENSOR_TYPE              REAL_SENSOR         // Real or virtual sensor. Can be REAL_SENSOR or VIRTUAL_SENSOR

#define MAX_APPLICATION_SPEED_RPM           4000                // Max speed for the current application in mechanical rpm
#define HALL_AVERAGING_FIFO_DEPTH           8                   // Depth of the FIFO used to average Hall sensor values

#define PID_TORQUE_KP_DEFAULT               300                 // Default gain if adaptive gain feature is not used
#define PID_FLUX_KP_DEFAULT                 100                 // Default gain if adaptive gain feature is not used
#define PID_FLUX_KI_DEFAULT                 1000                // Default gain if adaptive gain feature is not used

#define ENABLE_SPEED_LIMIT_CONTROL          true                // Enable or disable speed limit control
#define PID_SPEED_KP_DEFAULT                40                  // Default gain for speed control loop
#define PID_SPEED_KI_DEFAULT                4                   // Default gain for speed control loop
#define SP_KIDIV                            4096                // Speed control gain divider, to allow decimal value
#define SP_KIDIV_LOG                        LOG2(4096)          // Speed control gain divider log2, to allow decimal value

/************************** Power Limit Config  ************************/
#define ENABLE_LV_TORQUE_LIMIT              true               // Enable or disable the low voltage torque limit
#define LOW_VOLTAGE_THRESHOLD_PERCENTAGE    12                  // The threshold percentage of battery voltage before limiting torque
#define LOW_BATTERY_TORQUE                  150

#define ENABLE_MAX_POWER_LIMIT              true                // Enable or disable the foldback
#define MAX_TIME_BMS_TOLERANT               20000               // End time of derating for BMS protection in ms
#define MAX_POWER_LIMIT_TIMEOUT             10000               // Start time of derating for BMS protection in ms
#define MAX_BMS_POSITIVE_POWER              700                 // Maximum power at the end point of foldback
#define MAX_BMS_CONTINOUS_CURRENT           10                  // Maximum Power at the end point of foldback in amps

#define FOLDBACK_SPEED_INTERVAL             0                   // Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed
#define FOLDBACK_MOTOR_TEMP_INTERVAL        10                  // Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature

 /************************** Ramp Manager Config  ************************/
#define DEFAULT_TORQUE_SLOPE_UP             5000                // Slope in cNm per second
#define DEFAULT_TORQUE_SLOPE_DOWN           10000               // Slope in cNm per second
#define DEFAULT_SPEED_SLOPE_UP              10000               // Slope in #SPEED_UNIT per second
#define DEFAULT_SPEED_SLOPE_DOWN            10000               // Slope in #SPEED_UNIT per second

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA  16.91F              // Alpha constant to configure Butterworth filter for mechanical speed filtering
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA   -14.91F             // Beta constant to configure Butterworth filter for mechanical speed filtering

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM                       8                   // Number of motor pole pairs
#define RS                                  0.1f                // Stator resistance , ohm
#define LS                                  0.000235f           // Stator inductance, H   For I-PMSM it is equal to Lq
#define MOTOR_MAGNET_FLUX                   0.0195f             // Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests
#define MOTOR_VOLTAGE_CONSTANT              28.84f              // Volts RMS ph-ph /kRPM
#define ST_Torque_Coef                      1.2f                // this coeficient always keeps the starting torque higher than the nominal torque

#define PEAK_CURRENT_MOTOR_amps                   55                  // peak current in amps

#define MOTOR_MAX_SPEED_RPM                 2100                // Maximum rated speed - Old Example 2750 for 38Km/h

#define OV_TEMP_MOTOR_THRESHOLD_C           125                 // Maximum temperature in degree C
#define OV_TEMP_MOTOR_HYSTERESIS_C          5                   // Temperature to decrease after an overtemp fault occured before clearing the fault, in degree C
#define FLUX_WEAKENING_ENABLE               0                   // 0=disable 1=enable flux weakening , 

/***************** MOTOR SENSORS PARAMETERS  ******************************/
#define HALL_SENSORS_PLACEMENT              DEGREES_120         // Mechanical position of the sensors
                                                                // withreference to an electrical cycle.
                                                                // It can be either DEGREES_120 or DEGREES_60

#define HALL_PHASE_SHIFT                    60                  // Electrical phase shift in degree between the low to high
                                                                // transition of signal H1 and the zero crossing of the Bemf induced
                                                                // between phase A and B
#endif /* __DRIVE_PARAMETERS_R48_750W_H */
