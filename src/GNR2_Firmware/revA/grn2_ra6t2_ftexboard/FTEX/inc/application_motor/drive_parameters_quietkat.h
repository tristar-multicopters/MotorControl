/**
  * @file    drive_parameters_quietkat.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. This file is specific to quietkat motor.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_QUIETKAT_H
#define __DRIVE_PARAMETERS_QUIETKAT_H

/************************** Motor Config  ************************/
#define MOTOR_GEAR_RATIO                   (float)5             // Motor gear ratio, the value is always X turns of the motor
#define MOTOR_TEMP_SENSOR_TYPE              VIRTUAL_SENSOR      // Real or virtual sensor. Can be REAL_SENSOR or VIRTUAL_SENSOR

#define MAX_APPLICATION_SPEED_RPM           2000                // Max speed for the current application in mechanical rpm
#define HALL_AVERAGING_FIFO_DEPTH           10                  // depth of the FIFO used to

#define OV_VOLTAGE_THRESHOLD_V              70                  // Over-voltage threshold
#define UD_VOLTAGE_THRESHOLD_V              36                  // Under-voltage threshold - experimental value that prevents BMS shutdowns on dead battery

#define PID_TORQUE_KP_DEFAULT               600                 // Default gain if adaptative gain feature is not used
#define PID_FLUX_KP_DEFAULT                 600                 // Default gain if adaptative gain feature is not used
#define PID_FLUX_KI_DEFAULT                 3000                // Default gain if adaptative gain feature is not used

#define ENABLE_SPEED_LIMIT_CONTROL          false
#define PID_SPEED_KP_DEFAULT                100                 // Default gain speed control loop
#define PID_SPEED_KI_DEFAULT                10                  // Default gain speed control loop
#define SP_KIDIV                            16384               // Speed control gain divider, to allow decimal value
#define SP_KIDIV_LOG                        LOG2(16384)         // Speed control gain divider log2, to allow decimal value

/************************** Power Limit Config  ************************/
#define LOW_BATTERY_VOLTAGE_THRESHOLD      40                   // the threshold of battery voltage before limit torque

#define ENABLE_MAX_POWER_LIMIT              false               // to enable or disable the foldback
#define MAX_BMS_POSITIVE_POWER              500                 // Maximum Power at the end point of foldback
#define MAX_TIME_BMS_TOLERANT               20000               // End time of derating for BMS protection in ms
#define MAX_POWER_LIMIT_TIMEOUT             10000               // Start time of derating for BMS protection in ms

#define POWER_LIMIT_REF                     MAX_CURRENT_LIMIT   // defines if the code should use MAX_APPLICATION_POSITIVE_POWER or MAX_APPLICATION_CURRENT
#define MAX_APPLICATION_POSITIVE_POWER      750                 // Refers to maximum power in watts that drive can push to the motor
#define MAX_APPLICATION_NEGATIVE_POWER      750                 // Refers to maximum power in watts that drive can accept from the motor
#define MAX_APPLICATION_CURRENT             15                  // Refers to maximum battery current in amps that drive can accept from the motor

#define FOLDBACK_SPEED_INTERVAL             750                 // Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed
#define FOLDBACK_MOTOR_TEMP_INTERVAL        20                  // Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature

 /************************** Ramp Manager Config  ************************/
#define DEFAULT_TORQUE_SLOPE_UP             5000                // Slope in cNm per second
#define DEFAULT_TORQUE_SLOPE_DOWN           10000               // Slope in cNm per second
#define DEFAULT_SPEED_SLOPE_UP              500                 // Slope in #SPEED_UNIT per second
#define DEFAULT_SPEED_SLOPE_DOWN            500                 // Slope in #SPEED_UNIT per second

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA  7.366197724F        // Alpha constant to configure butterworth filter for mecanical speed filtering
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA  -5.366197724F        // Beta constant to configure butterworth filter for mecanical speed filtering

/************************** DUAL Specific parameters ************************/
#define MASTER_DEFAULT_TORQUE_SLOPE_UP      4000                // Slope in cNm per second
#define SLAVE_DEFAULT_TORQUE_SLOPE_UP       2000                // Slope in cNm per second - the  value is less in comparison to the master because the slave should

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM                       10                  // Number of motor pole pairs
#define RS                                  0.071f              // Stator resistance , ohm
#define LS                                  0.00015f            // Stator inductance, H   For I-PMSM it is equal to Lq
#define MOTOR_MAGNET_FLUX                   0.013f              // Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests
#define MOTOR_VOLTAGE_CONSTANT              24.5f               // Volts RMS ph-ph /kRPM
#define ST_Torque_Coef                      1.2f                // this coeficient always keeps the starting torque higher than the nominal torque

#define PEAK_CURRENT_amps                   55                  // peak current in amps
#define NOMINAL_TORQUE                      (uint16_t)(1.5 * 100 * POLE_PAIR_NUM * MOTOR_MAGNET_FLUX * PEAK_CURRENT_amps)    // Nominal torque to apply to motor in cNm   
                                                                                                                                                                                                 // Torque (cNm) = (3/2)* POLE_PAIR_NUM * MOTOR_MAGNET_FLUX * PEAK_CURRENT_amps
#define STARTING_TORQUE                     (uint16_t)(NOMINAL_TORQUE * ST_Torque_Coef)    // Maximum starting torque to apply to motor in cNm  Only used for Heavy bikes

#define MOTOR_MAX_SPEED_RPM                 1875                // Maximum rated speed

#define OV_TEMP_MOTOR_THRESHOLD_C           70                  // Maximum temperature in degree C
#define OV_TEMP_MOTOR_HYSTERESIS_C          10                  // Temperature to decrease after an overtemp fault occured before clearing the fault, in degree C
#define FLUX_WEAKENING_ENABLE               0                   // 0=disable 1=enable flux weakening , 

/***************** MOTOR SENSORS PARAMETERS  ******************************/
#define HALL_SENSORS_PLACEMENT    DEGREES_120                   // Mechanical position of the sensors
                                                                // withreference to an electrical cycle.
                                                                // It can be either DEGREES_120 or DEGREES_60

#define HALL_PHASE_SHIFT          60                            // Electrical phase shift in degree between the low to high
                                                                // transition of signal H1 and the zero crossing of the Bemf induced
                                                                // between phase A and B

#endif /* __DRIVE_PARAMETERS_QUIETKAT_H */
