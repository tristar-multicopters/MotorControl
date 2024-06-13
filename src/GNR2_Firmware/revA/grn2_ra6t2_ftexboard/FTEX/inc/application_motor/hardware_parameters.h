/**
  * @file    hardware_parameters.h
  * @brief   This file contains the general hardware parameters
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __HARDWARE_PARAMETERS_H
#define __HARDWARE_PARAMETERS_H

#define HARDWARE_EP1200       0
#define HARDWARE_EP600        1
#define HARDWARE_EP350        2
#define HARDWARE_EP700        3

#define MOTOR_NTC             0
#define HEATSINK_NTC          1

/****** NTC Parameters ******/
#define HEATSINK_NTC_BETA_COEFFICIENT         4100                       //Beta coefficient value as specified in the datasheet
#define HEATSINK_NTC_RATED_RESISTANCE         10000                      //NTC resistance at 25 degree celsius (in ohms).
#define HEATSINK_NTC_PULLDOWN_RESISTOR        10000                      // Resistance value of the pull-down resistor used in the heatsink NTC thermistor circuit (in ohms).
#define MOTOR_NTC_PULLUP_RESISTOR             10000                      // Resistance value of the pull-up resistor used in the motor NTC thermistor circuit (in ohms).
#define MOTOR_NTC_SERIES_PULLDOWN_RESISTOR    330                        // Resistance value of the series pull-down resistor used in the motor NTC thermistor circuit (in ohms).
#define HEATSINK_NTC_DRIFT_SLOPE              0.0904                     // Slope of the drift correction for the heatsink NTC thermistor.
#define HEATSINK_NTC_DRIFT_INTERCEPT          4.84                       // Intercept of the drift correction for the heatsink NTC thermistor.


#endif  /*__HARDWARE_PARAMETERS_H*/
