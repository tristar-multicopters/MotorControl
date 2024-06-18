/**
  * @file    motor_parameters_super73_500w.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a motor drive. This file is specific to the Super73 500W motor.
  
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
#ifndef __MOTOR_PARAMETERS_SUPER73_500W_H
#define __MOTOR_PARAMETERS_SUPER73_500W_H


/************************** Controller Config  ************************/


#define HARDWARE_SELECTION                 HARDWARE_EP700       // Controller selection to adapt controller parameters

/************************** Motor Config Parameters ************************/

#define MOTOR_GEAR_RATIO                   (float)6.74          // Motor gear ratio, the value is always X turns of the motor 
                                                                // Note: The gear ratio value mentioned in the datasheet is 5. 
                                                                // However, after the calculations, the gear ratio was found to be 6.74

#define MOTOR_TYPE                          HUB_DRIVE           // Motor type. Can be HUB_DRIVE or MID_DRIVE or DIRECT_DRIVE
#define POLE_PAIR_NUM                       8                   // Number of motor pole pairs
#define RS_VAL                              0.045f              // Stator resistance , ohm
#define LS                                  0.00007f            // Stator inductance, H   For I-PMSM it is equal to Lq
#define MOTOR_MAGNET_FLUX                   0.0151f             // Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests
#define MOTOR_VOLTAGE_CONSTANT              28.84f              // Volts RMS ph-ph /kRPM
#define ST_TORQUE_COEF                      1.2f                // this coeficient always keeps the starting torque higher than the nominal torque

#define PEAK_CURRENT_MOTOR_amps                66               // peak current in amps

/************************** Power Limit Parameters ************************/

#define ESTIMATED_EFFICIENCY                 80              // Percent efficiency of input compared to output power. Default value 95%.

/************************** Speed Limit Parameters ************************/

#define MAX_APPLICATION_SPEED_RPM           4000                // Max speed for the current application in mechanical rpm

#define ENABLE_SPEED_LIMIT_CONTROL          true                // Enable or disable speed limit control
#define PID_SPEED_KP_DEFAULT                100                  // Default gain for speed control loop
#define PID_SPEED_KI_DEFAULT                4                   // Default gain for speed control loop
#define SP_KIDIV                            4096                // Speed control gain divider, to allow decimal value
#define SP_KIDIV_LOG                        LOG2(4096)          // Speed control gain divider log2, to allow decimal value

#define FOLDBACK_SPEED_INTERVAL             0                   // Speed interval (#SPEED_UNIT) of the decreasing torque ramp to limit speed

/************************** Torque Limit Parameters ************************/

#define PID_TORQUE_KP_DEFAULT               300                 // Default gain if adaptive gain feature is not used

/************************** Flux Parameters ************************/

#define PID_FLUX_KP_DEFAULT                 100                 // Default gain if adaptive gain feature is not used
#define PID_FLUX_KI_DEFAULT                 1000                // Default gain if adaptive gain feature is not used

#define FLUX_WEAKENING_ENABLE               false               // enable or disable flux weakening
#define FLUX_DIRECTION                      1               // direction of demagnetization current  1 clockwise, -1 counter clockwise
/************************** Ramp Manager Parameters  ************************/

#define DEFAULT_TORQUE_SLOPE_UP             5000                // Slope in cNm per second
#define DEFAULT_TORQUE_SLOPE_DOWN           5000               // Slope in cNm per second
#define DEFAULT_SPEED_SLOPE_UP              10000               // Slope in #SPEED_UNIT per second
#define DEFAULT_SPEED_SLOPE_DOWN            10000               // Slope in #SPEED_UNIT per second

#define MEC_SPEED_FILTER_BUTTERWORTH_ALPHA  16.91F              // Alpha constant to configure Butterworth filter for mechanical speed filtering
#define MEC_SPEED_FILTER_BUTTERWORTH_BETA   -14.91F             // Beta constant to configure Butterworth filter for mechanical speed filtering

/************************** Temp Limit Parameters  ************************/

#define MOTOR_TEMP_SENSOR_TYPE              VIRTUAL_SENSOR      // Real or virtual sensor. Can be REAL_SENSOR or VIRTUAL_SENSOR
#define MOTOR_TEMP_MIXED                    false               // true if the motor temperature signal is mixed with wheelspeed.
    
#define OV_TEMP_MOTOR_THRESHOLD_C           110                 // Maximum temperature in degree C
#define OV_TEMP_MOTOR_HYSTERESIS_C          5                   // Temperature to decrease after an overtemp fault occurred before clearing the fault, in degree C
#define FOLDBACK_MOTOR_TEMP_INTERVAL        10                  // Temperature interval (degree C) of the decreasing torque ramp to limit motor temperature

/************************** Hall Sensor Parameters ******************************/

#define HALL_SENSORS_PLACEMENT              DEGREES_120         // Mechanical position of the sensors
                                                                // withreference to an electrical cycle.
                                                                // It can be either DEGREES_120 or DEGREES_60
#define HALL_AVERAGING_FIFO_DEPTH           8                   // Depth of the FIFO used to average Hall sensor values
#define HALL_PHASE_SHIFT                    60                  // Electrical phase shift in degrees between the low to high transition of signal H1 and the zero crossing of the Bemf induced between phase A and B

#define EN_VIBRATION_ERROR                  false               // Enable or disable motor vibration error

/************** Wheel Speed Sensor Parameters  *****************************/

#define MOTOR_WSS_NBR_PER_ROTATION          6                   // Wheel speed sensor cycle number for one wheel rotation

/************** Current and Speed PID Parameters  *****************************/

#define CURRENT_PID_LUT_SPEED_1             300
#define CURRENT_PID_LUT_SPEED_2             700

#define IQ_KP_VS_SPEED_1                    60
#define IQ_KP_VS_SPEED_2                    60

#define IQ_KI_VS_SPEED_1                    20
#define IQ_KI_VS_SPEED_2                    20

#define ID_KP_VS_SPEED_1                    50
#define ID_KP_VS_SPEED_2                    50

#define ID_KI_VS_SPEED_1                    20
#define ID_KI_VS_SPEED_2                    20


#endif /* __MOTOR_PARAMETERS_SUPER73_500W_H */
