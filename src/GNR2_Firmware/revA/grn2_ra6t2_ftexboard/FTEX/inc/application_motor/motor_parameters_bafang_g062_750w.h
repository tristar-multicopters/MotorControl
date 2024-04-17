/**
  * @file    motor_parameters_bafang_g062_750w.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. This file is specific to the Bafang G062 750W motor.
  
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
#ifndef __MOTOR_PARAMETERS_BAFANG_G062_750W_H
#define __MOTOR_PARAMETERS_BAFANG_G062_750W_H

/************************** Controller Config  ************************/

#define HARDWARE_SELECTION                 HARDWARE_EP700       // Controller selection to adapt controller parameters

/************************** Motor Config Parameters ************************/

#define MOTOR_GEAR_RATIO                    (float)5            // Motor gear ratio, the value is always X turns of the motor 
#define MOTOR_TYPE                          HUB_DRIVE           // Motor type. Can be HUB_DRIVE or MID_DRIVE or DIRECT_DRIVE
#define POLE_PAIR_NUM                       10                  // Number of motor pole pairs
#define RS_VAL                              0.071f              // Stator resistance , ohm
#define LS                                  0.00015f            // Stator inductance, H   For I-PMSM it is equal to Lq
#define MOTOR_MAGNET_FLUX                   0.013f              // Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests
#define MOTOR_VOLTAGE_CONSTANT              24.5f               // Volts RMS ph-ph /kRPM
#define ST_TORQUE_COEF                      1.2f                // this coeficient always keeps the starting torque higher than the nominal torque

#define PEAK_CURRENT_MOTOR_amps                55               // peak current in amps

/************************** Power Limit Parameters ************************/

#define ESTIMATED_EFFICIENCY                100                 // Percent efficiency of input compared to output power

/************************** Speed Limit Parameters ************************/

#define MAX_APPLICATION_SPEED_RPM           2000                // Max speed for the current application in mechanical rpm

#define ENABLE_SPEED_LIMIT_CONTROL          false
#define PID_SPEED_KP_DEFAULT                100                 // Default gain speed control loop
#define PID_SPEED_KI_DEFAULT                10                  // Default gain speed control loop
#define SP_KIDIV                            16384               // Speed control gain divider, to allow decimal value
#define SP_KIDIV_LOG                        LOG2(16384)         // Speed control gain divider log2, to allow decimal value

#define FOLDBACK_SPEED_INTERVAL             750                 // Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed

/************************** Torque Limit Parameters ************************/

#define PID_TORQUE_KP_DEFAULT               600                 // Default gain if adaptative gain feature is not used

/************************** Flux Parameters ************************/

#define PID_FLUX_KP_DEFAULT                 600                 // Default gain if adaptative gain feature is not used
#define PID_FLUX_KI_DEFAULT                 3000                // Default gain if adaptative gain feature is not used

#define FLUX_WEAKENING_ENABLE               false               // enable or disable flux weakening

/************************** Ramp Manager Parameters  ************************/

#define DEFAULT_TORQUE_SLOPE_UP             5000                // Slope in cNm per second
#define DEFAULT_TORQUE_SLOPE_DOWN           10000               // Slope in cNm per second
#define DEFAULT_SPEED_SLOPE_UP              500                 // Slope in #SPEED_UNIT per second
#define DEFAULT_SPEED_SLOPE_DOWN            500                 // Slope in #SPEED_UNIT per second

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA  7.366197724F        // Alpha constant to configure butterworth filter for mecanical speed filtering
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA  -5.366197724F        // Beta constant to configure butterworth filter for mecanical speed filtering

 /************************** Temp Limit Parameters  ************************/

#define MOTOR_TEMP_SENSOR_TYPE              VIRTUAL_SENSOR      // Real or virtual sensor. Can be REAL_SENSOR or VIRTUAL_SENSOR
#define MOTOR_TEMP_MIXED                    false               // true if the motor temperature signal is mixed with wheelspeed.
    
#define OV_TEMP_MOTOR_THRESHOLD_C           70                  // Maximum temperature in degree C
#define OV_TEMP_MOTOR_HYSTERESIS_C          10                  // Temperature to decrease after an overtemp fault occurred before clearing the fault, in degree C
#define FOLDBACK_MOTOR_TEMP_INTERVAL        20                  // Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature

/************************** Hall Sensor Parameters ******************************/

#define HALL_SENSORS_PLACEMENT              DEGREES_120         // Mechanical position of the sensors
                                                                // withreference to an electrical cycle.
                                                                // It can be either DEGREES_120 or DEGREES_60
#define HALL_AVERAGING_FIFO_DEPTH           10                  // Depth of the FIFO used to
#define HALL_PHASE_SHIFT                    60                  // Electrical phase shift in degrees between the low to high transition of signal H1 and the zero crossing of the Bemf induced between phase A and B

/************** Wheel Speed Sensor Parameters  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION     1               // Wheel speed sensor cycle number for one wheel rotation

/************************** DUAL Specific parameters ************************/

#define MASTER_DEFAULT_TORQUE_SLOPE_UP      4000                // Slope in cNm per second
#define SLAVE_DEFAULT_TORQUE_SLOPE_UP       2000                // Slope in cNm per second - the  value is less in comparison to the master because the slave should

/************** Current and Speed PID Parameters  *****************************/

#define IQ_KP_VS_SPEED_1                    90
#define IQ_KP_VS_SPEED_2                    300

#define IQ_KI_VS_SPEED_1                    27
#define IQ_KI_VS_SPEED_2                    200

#define ID_KP_VS_SPEED_1                    87
#define ID_KP_VS_SPEED_2                    300

#define ID_KI_VS_SPEED_1                    26
#define ID_KI_VS_SPEED_2                    200


#endif /* __MOTOR_PARAMETERS_BAFANG_G062_750W_H */
