/**
  * @file    power_stage_parameters.h
  * @brief   This file contains the parameters needed for the Motor Control application
  *          in order to configure a power stage.
  *
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWER_STAGE_PARAMETERS_H
#define __POWER_STAGE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/************* PWM Driving signals section **************/

#define HW_DEAD_TIME_NS               100 /*!< Dead-time inserted
                                                         by HW if low side signals
                                                         are not used */

/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR      0.0354 /*!< It expresses how
                                                       much the Vbus is attenuated
                                                       before being converted into
                                                       digital value */
#define NOMINAL_BUS_VOLTAGE_V         48

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN            0.0165

/*** Noise parameters ***/
#define TNOISE_NS                     2550
#define TRISE_NS                      2550
#define MAX_TNTR_NS TRISE_NS

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                          1.767 /*!< in Volts */
#define T0_C                          30 /*!< in Celsius degrees */
#define dV_dT                         0.030 /*!< V/Celsius degrees */
#define T_MAX                         70 /*!< Sensor measured
                                                       temperature at maximum
                                                       power stage working
                                                       temperature, Celsius degrees */

#endif /*__POWER_STAGE_PARAMETERS_H*/


