/**
  * @file    pmsm_motor_parameters_wheel.h
  * @brief   This file contains the parameters needed in order to configure the motor of wheel motor.
  *
*/

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __PMSM_MOTOR_PARAMETERS_WHEEL_H
#define __PMSM_MOTOR_PARAMETERS_WHEEL_H

/************************
 *** Motor Parameters ***
 ************************/

 /***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          8 /* Number of motor pole pairs */
#define RS                     0.1f /* Stator resistance , ohm*/
#define LS                     0.0001f /* Stator inductance, H   For I-PMSM it is equal to Lq */
#define MOTOR_MAGNET_FLUX       0.022f  /* Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests */
#define MOTOR_VOLTAGE_CONSTANT  32.0f   /*!< Volts RMS ph-ph /kRPM */
#define ST_Torque_Coef          1.2f       /* this coeficient always keeps the starting torque higher than the nominal torque */

#define PEAK_CURRENT_amps       55      /* peak current in amps     */
#define NOMINAL_TORQUE          (uint16_t)(1.5 * 100 * POLE_PAIR_NUM * MOTOR_MAGNET_FLUX * PEAK_CURRENT_amps)    /* Nominal torque to apply to motor in cNm   
                                                                                                 Torque (cNm) = (3/2)* POLE_PAIR_NUM * MOTOR_MAGNET_FLUX * PEAK_CURRENT_amps */
#define STARTING_TORQUE         (uint16_t)(NOMINAL_TORQUE * ST_Torque_Coef)    /* Maximum starting torque to apply to motor in cNm  Only used for Heavy bikes*/


#define MOTOR_MAX_SPEED_RPM     2100   /*!< Maximum rated speed  */

#define ID_DEMAG_amps                -5 /*!< Demagnetization current */
#define MOTOR_MAX_TEMPERATURE_C 70    /* Maximum temperature in degree C */
#define FLUX_WEAKENING_ENABLE   0       /* 0=disable 1=enable flux weakening , 

/***************** MOTOR SENSORS PARAMETERS  ******************************/

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!< Mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        60 /*!< Electrical phase shift in degree between the low to high
                                        transition of signal H1 and the zero crossing of the Bemf induced
                                        between phase A and B */

																						
#endif																						

