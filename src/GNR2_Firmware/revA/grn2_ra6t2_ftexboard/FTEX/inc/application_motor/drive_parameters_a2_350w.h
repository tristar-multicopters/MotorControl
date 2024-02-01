/**
  * @file    drive_parameters_velec_a2.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. This file is specific to velec A2 motor.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_VELEC_A2_H
#define __DRIVE_PARAMETERS_VELEC_A2_H

/************************** Motor Config  ************************/
#define MOTOR_GEAR_RATIO                    (float)10.9890  // Motor gear ratio, the value is always X turns of the motor
#define MOTOR_TEMP_SENSOR_TYPE              VIRTUAL_SENSOR  // Real or virtual sensor. Can be REAL_SENSOR or VIRTUAL_SENSOR

#define MAX_APPLICATION_SPEED_RPM           4000            // Max speed for the current application in mechanical rpm
#define HALL_AVERAGING_FIFO_DEPTH           8               // Depth of the FIFO used to

#define OV_VOLTAGE_THRESHOLD_V              75              // Over-voltage threshold
#define UD_VOLTAGE_THRESHOLD_V              32              // Under-voltage threshold

#define PID_TORQUE_KP_DEFAULT               300             // Default gain if adaptive gain feature is not used
#define PID_FLUX_KP_DEFAULT                 100             // Default gain if adaptive gain feature is not used
#define PID_FLUX_KI_DEFAULT                 1000            // Default gain if adaptive gain feature is not used

#define ENABLE_SPEED_LIMIT_CONTROL          false
#define PID_SPEED_KP_DEFAULT                100             // Default gain speed control loop
#define PID_SPEED_KI_DEFAULT                10              // Default gain speed control loop
#define SP_KIDIV                            16384           // Speed control gain divider, to allow decimal value
#define SP_KIDIV_LOG                        LOG2(16384)     // Speed control gain divider log2, to allow decimal value

/************************** Power Limit Config  ************************/
#define LOW_BATTERY_VOLTAGE_THRESHOLD       35              // The threshold of battery voltage before limit torque
#define LOW_BATTERY_TORQUE                  150

#define ENABLE_MAX_POWER_LIMIT              true            // To enable or disable the foldback
#define MAX_TIME_BMS_TOLERANT               20000           // End time of derating for BMS protection in ms
#define MAX_POWER_LIMIT_TIMEOUT             10000           // Start time of derating for BMS protection in ms
#define MAX_BMS_POSITIVE_POWER              500             // Maximum Power at the end point of foldback
#define MAX_BMS_CONTINOUS_CURRENT           10              // Maximum Power at the end point of foldback in amps

#define POWER_LIMIT_REF                     MAX_POWER_LIMIT // Defines if the code should use MAX_APPLICATION_POSITIVE_POWER or MAX_APPLICATION_CURRENT
#define MAX_APPLICATION_POSITIVE_POWER      700             // Refers to maximum power in watts that drive can push to the motor
#define MAX_APPLICATION_NEGATIVE_POWER      700             // Refers to maximum power in watts that drive can accept from the motor
#define MAX_APPLICATION_CURRENT             22              // Refers to maximum current in amps that drive can accept from the motor

#define FOLDBACK_SPEED_INTERVAL             0               // 750 Removed to let VC control top speed. Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed
#define FOLDBACK_MOTOR_TEMP_INTERVAL        20              // Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature

 /************************** Ramp Manager Config  ************************/
#define DEFAULT_TORQUE_SLOPE_UP             3000            // Slope in cNm per second
#define DEFAULT_TORQUE_SLOPE_DOWN           10000           // Slope in cNm per second
#define DEFAULT_SPEED_SLOPE_UP              500             // Slope in #SPEED_UNIT per second
#define DEFAULT_SPEED_SLOPE_DOWN            500             // Slope in #SPEED_UNIT per second

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA  16.91F          // Alpha constant to configure butterworth filter for mechanical speed filtering
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA   -14.91F         // Beta constant to configure butterworth filter for mechanical speed filtering

/************************** MOTOR ELECTRICAL PARAMETERS ******************************/
#define POLE_PAIR_NUM                       8               // Number of motor pole pairs
#define RS                                  0.1f            // Stator resistance, ohm
#define LS                                  0.000235f       // Stator inductance, H (For I-PMSM it is equal to Lq)
#define MOTOR_MAGNET_FLUX                   0.0195f         // Flux of Permanent magnets used in the motor, derived by performing motor tests
#define MOTOR_VOLTAGE_CONSTANT              28.84f          // Volts RMS ph-ph /kRPM
#define ST_Torque_Coef                      1.2f            // Coefficient to keep the starting torque higher than the nominal torque

#define PEAK_CURRENT_amps                   50              // Peak current in amps
#define NOMINAL_TORQUE                      (uint16_t)(1.5 * 100 * POLE_PAIR_NUM * MOTOR_MAGNET_FLUX * PEAK_CURRENT_amps)   // Nominal torque to apply to motor in cNm
                                                                                        // Torque (cNm) = (3/2) * POLE_PAIR_NUM * MOTOR_MAGNET_FLUX * PEAK_CURRENT_amps
#define STARTING_TORQUE                     (uint16_t)(NOMINAL_TORQUE * ST_Torque_Coef) // Maximum starting torque to apply to motor in cNm (Only used for Heavy bikes)

#define MOTOR_MAX_SPEED_RPM                 2100            // Maximum rated speed - Old Example 2750 for 38Km/h                                                      

#define OV_TEMP_MOTOR_THRESHOLD_C           100             // Maximum temperature in degree C
#define OV_TEMP_MOTOR_HYSTERESIS_C          10              // Temperature to decrease after an overtemp fault occurred before clearing the fault, in degree C
#define FLUX_WEAKENING_ENABLE               0               // 0=disable 1=enable flux weakening

/************************** MOTOR SENSORS PARAMETERS  ******************************/
#define HALL_SENSORS_PLACEMENT              DEGREES_120     // Mechanical position of the sensors
                                                            // withreference to an electrical cycle.
                                                            // It can be either DEGREES_120 or DEGREES_60

#define HALL_PHASE_SHIFT                    60              // Electrical phase shift in degree between the low to high
                                                            // transition of signal H1 and the zero crossing of the Bemf induced
                                                            // between phase A and B
                                                     													
#endif /* __DRIVE_PARAMETERS_VELEC_A2_H */
