/**
  * @file    motor_parameters_bafang_g020_500w.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. This file is specific to AKM 128SX 750W motor.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_PARAMETERS_BAFANG_G020_500W_H
#define __MOTOR_PARAMETERS_BAFANG_G020_500W_H

/************************** Controller Config  ************************/

#define HARDWARE_SELECTION                 HARDWARE_EP700    // Controller selection to adapt controller parameters
/************************** Motor Config  ************************/
#define MOTOR_GEAR_RATIO                    (float)10.9890      // Motor gear ratio, the value is always X turns of the motor 
#define MOTOR_TYPE                          HUB_DRIVE           // Motor type. Can be HUB_DRIVE or MID_DRIVE
#define MOTOR_TEMP_SENSOR_TYPE              VIRTUAL_SENSOR         // Real or virtual sensor. Can be REAL_SENSOR or VIRTUAL_SENSOR
#define MOTOR_TEMP_MIXED                    false               // true if the motor temperature signal is mixed with wheelspeed.

#define MAX_APPLICATION_SPEED_RPM           1500                // Max speed for the current application in mechanical rpm
#define HALL_AVERAGING_FIFO_DEPTH           8                   // Depth of the FIFO used to average Hall sensor values

#define PID_TORQUE_KP_DEFAULT               300                 // Default gain if adaptive gain feature is not used
#define PID_FLUX_KP_DEFAULT                 300                 // Default gain if adaptive gain feature is not used
#define PID_FLUX_KI_DEFAULT                 50                // Default gain if adaptive gain feature is not used

#define ENABLE_SPEED_LIMIT_CONTROL          true                // Enable or disable speed limit control
#define PID_SPEED_KP_DEFAULT                100                  // Default gain for speed control loop
#define PID_SPEED_KI_DEFAULT                10                   // Default gain for speed control loop
#define SP_KIDIV                            256                // Speed control gain divider, to allow decimal value
#define SP_KIDIV_LOG                        LOG2(256)          // Speed control gain divider log2, to allow decimal value

/************************** Power Limit Config  ************************/
#define ENABLE_LV_TORQUE_LIMIT              false               // Enable or disable the low voltage torque limit
#define LOW_VOLTAGE_THRESHOLD_PERCENTAGE    12                  // The threshold percentage of battery voltage before limiting torque
#define LOW_BATTERY_TORQUE                  150

#define ENABLE_MAX_POWER_LIMIT              false                // Enable or disable the foldback
#define MAX_TIME_BMS_TOLERANT               20000               // End time of derating for BMS protection in ms
#define MAX_POWER_LIMIT_TIMEOUT             10000               // Start time of derating for BMS protection in ms
#define MAX_BMS_POSITIVE_POWER              500                 // Maximum power at the end point of foldback
#define MAX_BMS_CONTINOUS_CURRENT           10                  // Maximum Power at the end point of foldback in amps

#define ESTIMATED_EFFICIENCY                100             // Percent efficiency of input compared to output power

#define FOLDBACK_SPEED_INTERVAL             0                   // Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed
#define FOLDBACK_MOTOR_TEMP_INTERVAL        20                  // Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature

 /************************** Ramp Manager Config  ************************/
#define DEFAULT_TORQUE_SLOPE_UP             2700                // Slope in cNm per second
#define DEFAULT_TORQUE_SLOPE_DOWN           5000               // Slope in cNm per second
#define DEFAULT_SPEED_SLOPE_UP              500               // Slope in #SPEED_UNIT per second
#define DEFAULT_SPEED_SLOPE_DOWN            500               // Slope in #SPEED_UNIT per second

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA  7.366197724F             // Alpha constant to configure Butterworth filter for mechanical speed filtering
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA   -5.366197724F            // Beta constant to configure Butterworth filter for mechanical speed filtering

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM                       8                   // Number of motor pole pairs
#define RS                                  0.071f                // Stator resistance , ohm
#define LS                                  0.00015f           // Stator inductance, H   For I-PMSM it is equal to Lq
#define MOTOR_MAGNET_FLUX                   0.0175f             // Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests
#define MOTOR_VOLTAGE_CONSTANT              22.5f              // Volts RMS ph-ph /kRPM
#define ST_Torque_Coef                      1.2f                // this coeficient always keeps the starting torque higher than the nominal torque

#define PEAK_CURRENT_MOTOR_amps             55                  // peak current in amps

#define MOTOR_MAX_SPEED_RPM                 1200                // Maximum rated speed - Old Example 2750 for 38Km/h

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

/************** WHEEL SPEED SENSOR PARAMETERS  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION     4       // Wheel speed sensor cycle number for one wheel rotation
                                                            
#endif /* __MOTOR_PARAMETERS_BAFANG_G020_500W_H */
