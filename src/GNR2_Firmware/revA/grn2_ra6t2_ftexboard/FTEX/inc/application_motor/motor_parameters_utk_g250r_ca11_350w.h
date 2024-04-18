/**
  * @file    motor_parameters_utk_g250r_ca11_350w.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. This file is specific to the UTK G250R CA11 350W motor.
  
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
#ifndef __MOTOR_PARAMETERS_UTK_G250R_CA11_350W_H
#define __MOTOR_PARAMETERS_UTK_G250R_CA11_350W_H

/************************** Controller Config  ************************/

#define HARDWARE_SELECTION                 HARDWARE_EP700       // Controller selection to adapt controller parameters

/************************** Motor Config Parameters ************************/
#define MOTOR_GEAR_RATIO                    (float)4.3333       // Motor gear ratio, the value is always X turns of the motor
#define MOTOR_TYPE                          HUB_DRIVE           // Motor type. Can be HUB_DRIVE or MID_DRIVE or DIRECT_DRIVE
#define POLE_PAIR_NUM                       8                   // Number of motor pole pairs
#define RS_VAL                              0.1f                // Stator resistance , ohm
#define LS                                  0.000235f           // Stator inductance, H   For I-PMSM it is equal to Lq
#define MOTOR_MAGNET_FLUX                   0.01f               // Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests
#define MOTOR_VOLTAGE_CONSTANT              28.84f              // Volts RMS ph-ph /kRPM
#define ST_TORQUE_COEF                      1.2f                // this coeficient always keeps the starting torque higher than the nominal torque

#define PEAK_CURRENT_MOTOR_amps             50                  // peak current in amps

/************************** Power Limit Parameters ************************/

#define ESTIMATED_EFFICIENCY                100                 // Percent efficiency of input compared to output power

/************************** Speed Limit Parameters ************************/

#define MAX_APPLICATION_SPEED_RPM           4000                // Max speed for the current application in mechanical rpm

#define ENABLE_SPEED_LIMIT_CONTROL          false
#define PID_SPEED_KP_DEFAULT                100                 // Default gain speed control loop
#define PID_SPEED_KI_DEFAULT                10                  // Default gain speed control loop
#define SP_KIDIV                            16384               // Speed control gain divider, to allow decimal value
#define SP_KIDIV_LOG                        LOG2(16384)         // Speed control gain divider log2, to allow decimal value

#define FOLDBACK_SPEED_INTERVAL             0                   // 750 Removed to let VC control top speed. Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed

/************************** Torque Limit Parameters ************************/

#define PID_TORQUE_KP_DEFAULT               300                 // Default gain if adaptive gain feature is not used

/************************** Flux Parameters ************************/

#define PID_FLUX_KP_DEFAULT                 100                 // Default gain if adaptive gain feature is not used
#define PID_FLUX_KI_DEFAULT                 1000                // Default gain if adaptive gain feature is not used

#define FLUX_WEAKENING_ENABLE               false               // enable or disable flux weakening

/************************** Ramp Manager Parameters  ************************/

#define DEFAULT_TORQUE_SLOPE_UP             2500                // Slope in cNm per second
#define DEFAULT_TORQUE_SLOPE_DOWN           10000               // Slope in cNm per second
#define DEFAULT_SPEED_SLOPE_UP              500                 // Slope in #SPEED_UNIT per second
#define DEFAULT_SPEED_SLOPE_DOWN            500                 // Slope in #SPEED_UNIT per second

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA  16.91F              // Alpha constant to configure butterworth filter for mechanical speed filtering
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA  -14.91F              // Beta constant to configure butterworth filter for mechanical speed filtering

/************************** Temp Limit Parameters  ************************/

#define MOTOR_TEMP_SENSOR_TYPE              REAL_SENSOR         // Real or virtual sensor. Can be REAL_SENSOR or VIRTUAL_SENSOR
#define MOTOR_TEMP_MIXED                    false               // true if the motor temperature signal is mixed with wheelspeed.
    
#define OV_TEMP_MOTOR_THRESHOLD_C           70                  // Maximum temperature in degree C
#define OV_TEMP_MOTOR_HYSTERESIS_C          10                  // Temperature to decrease after an overtemp fault occured before clearing the fault, in degree C
#define FOLDBACK_MOTOR_TEMP_INTERVAL        20                  // Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature

/************************** Hall Sensor Parameters ******************************/

#define HALL_SENSORS_PLACEMENT              DEGREES_120         // Mechanical position of the sensors
                                                                // withreference to an electrical cycle.
                                                                // It can be either DEGREES_120 or DEGREES_60
#define HALL_AVERAGING_FIFO_DEPTH           8                   // Depth of the FIFO used to average Hall sensor values
#define HALL_PHASE_SHIFT                    60                  // Electrical phase shift in degrees between the low to high transition of signal H1 and the zero crossing of the Bemf induced between phase A and B

/************** Wheel Speed Sensor Parameters  *****************************/

#define WHEEL_SPEED_SENSOR_NBR_PER_ROTATION     6               // Wheel speed sensor cycle number for one wheel rotation

/************** Current and Speed PID Parameters  *****************************/

#define IQ_KP_VS_SPEED_1                    262
#define IQ_KP_VS_SPEED_2                    262

#define IQ_KI_VS_SPEED_1                    40
#define IQ_KI_VS_SPEED_2                    20

#define ID_KP_VS_SPEED_1                    171
#define ID_KP_VS_SPEED_2                    171

#define ID_KI_VS_SPEED_1                    40
#define ID_KI_VS_SPEED_2                    20


#endif /* __MOTOR_PARAMETERS_UTK_G250R_CA11_350W_H */
