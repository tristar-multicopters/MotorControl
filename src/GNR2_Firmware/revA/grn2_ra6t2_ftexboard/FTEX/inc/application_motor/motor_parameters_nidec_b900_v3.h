/**
  * @file    motor_parameters_nidec_b900_v3.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. 
  *          This file is specific to the B900 v3 motor.
  
  
      ::::    :::       ::::::::   :::::::::::       :::::::::: 
     :+:+:   :+:      :+:    :+:      :+:           :+:         
    :+:+:+  +:+      +:+    +:+      +:+           +:+          
   +#+ +:+ +#+      +#+    +:+      +#+           +#++:++#      
  +#+  +#+#+#      +#+    +#+      +#+           +#+            
 #+#   #+#+#      #+#    #+#      #+#           #+#             
###    ####       ########       ###           ##########          
               
When you add parameters to this file, you MUST create a variable within MotorParamters_t in motor_parameters.h.
Use the variable in MotorParameters_t AND NOT THE DEFINE.
These variables are used to configure the motor within the smart config tool.

*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_PARAMETERS_NIDEC_B900_V3_H
#define __MOTOR_PARAMETERS_NIDEC_B900_V3_H

/************************** Controller Config  ************************/

#define HARDWARE_SELECTION                 HARDWARE_EP700    // Controller selection to adapt controller parameters
/************************** Motor Config  ************************/
#define MOTOR_GEAR_RATIO                    (float)36       // Motor gear ratio, the value is always X turns of the motor 
#define MOTOR_TYPE                          MID_DRIVE
#define MOTOR_TEMP_SENSOR_TYPE              REAL_SENSOR     // Real or virtual sensor. Can be REAL_SENSOR or VIRTUAL_SENSOR
#define MOTOR_TEMP_MIXED                    false           // true if the motor temperature signal is mixed with wheelspeed.

#define MAX_APPLICATION_SPEED_RPM           3600            // Max speed for the current application in mechanical rpm
#define HALL_AVERAGING_FIFO_DEPTH           8               // Depth of the FIFO used to

#define PID_TORQUE_KP_DEFAULT               300             // Default gain if adaptative gain feature is not used
#define PID_FLUX_KP_DEFAULT                 100             // Default gain if adaptative gain feature is not used
#define PID_FLUX_KI_DEFAULT                 1000            // Default gain if adaptative gain feature is not used

#define ENABLE_SPEED_LIMIT_CONTROL          true
#define PID_SPEED_KP_DEFAULT                100             // Default gain speed control loop
#define PID_SPEED_KI_DEFAULT                10              // Default gain speed control loop
#define SP_KIDIV                            16384           // Speed control gain divider, to allow decimal value
#define SP_KIDIV_LOG                        LOG2(16384)     // Speed control gain divider log2, to allow decimal value

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM                       6               // Number of motor pole pairs
#define RS_VAL                              0.044f          // Stator resistance , ohm
#define LS                                  0.000235f       // Stator inductance, H   For I-PMSM it is equal to Lq
#define MOTOR_MAGNET_FLUX                   0.0056f          // Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests
#define MOTOR_VOLTAGE_CONSTANT              28.84f          // Volts RMS ph-ph /kRPM
#define ST_TORQUE_COEF                      1.2f            // this coeficient always keeps the starting torque higher than the nominal torque

#define PEAK_CURRENT_MOTOR_amps             49         // peak current in amps

#define OV_TEMP_MOTOR_THRESHOLD_C           150             // Maximum temperature in degree C
#define OV_TEMP_MOTOR_HYSTERESIS_C          5              // Temperature to decrease after an overtemp fault occured before clearing the fault, in degree C
#define FLUX_WEAKENING_ENABLE               false               // enable or disable flux weakening
    
/************************** Power Limit Config  ************************/
#define ENABLE_LV_TORQUE_LIMIT              false           // Enable or disable the low voltage torque limit
#define LOW_VOLTAGE_THRESHOLD_PERCENTAGE    10              // The threshold percentage of battery voltage before limiting torque
#define LOW_BATTERY_TORQUE                  150

#define ENABLE_MAX_POWER_LIMIT              true            // To enable or disable the foldback
#define MAX_TIME_BMS_TOLERANT               20000           // End time of derating for BMS protection in ms
#define MAX_POWER_LIMIT_TIMEOUT             10000           // Start time of derating for BMS protection in ms
#define MAX_BMS_POSITIVE_POWER              750             // Maximum Power at the end point of foldback
#define MAX_BMS_CONTINUOUS_CURRENT           10              // Maximum Power at the end point of foldback in amps

#define ESTIMATED_EFFICIENCY                67              // Percent efficiency of input compared to output power

#define FOLDBACK_SPEED_INTERVAL             750             // Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed
#define FOLDBACK_MOTOR_TEMP_INTERVAL        45              // Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature

 /************************** Ramp Manager Config  ************************/
#define DEFAULT_TORQUE_SLOPE_UP              3000           // Slope in cNm per second
#define DEFAULT_TORQUE_SLOPE_DOWN            3000           // Slope in cNm per second
#define DEFAULT_SPEED_SLOPE_UP              25000           // Slope in #SPEED_UNIT per second
#define DEFAULT_SPEED_SLOPE_DOWN            25000           // Slope in #SPEED_UNIT per second

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA  16.91F          // Alpha constant to configure butterworth filter for mecanical speed filtering
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA  -14.91F          // Beta constant to configure butterworth filter for mecanical speed filtering

/***************** MOTOR SENSORS PARAMETERS  ******************************/
#define HALL_SENSORS_PLACEMENT              DEGREES_120     // Mechanical position of the sensors
                                                            // withreference to an electrical cycle.
                                                            // It can be either DEGREES_120 or DEGREES_60

#define HALL_PHASE_SHIFT                    60              // Electrical phase shift in degree between the low to high
                                                            // transition of signal H1 and the zero crossing of the Bemf induced
                                                            // between phase A and B

/************** WHEEL SPEED SENSOR PARAMETERS  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION     1       // Wheel speed sensor cycle number for one wheel rotation

/************** CURRENT AND SPEED PID PARAMETERS PARAMETERS  *****************************/

#define IQ_KP_VS_SPEED_1                    900
#define IQ_KP_VS_SPEED_2                    900

#define IQ_KI_VS_SPEED_1                    10
#define IQ_KI_VS_SPEED_2                    10

#define ID_KP_VS_SPEED_1                    80
#define ID_KP_VS_SPEED_2                    80

#define ID_KI_VS_SPEED_1                    10
#define ID_KI_VS_SPEED_2                    10


#endif /* __MOTOR_PARAMETERS_NIDEC_B900_V3_H */
