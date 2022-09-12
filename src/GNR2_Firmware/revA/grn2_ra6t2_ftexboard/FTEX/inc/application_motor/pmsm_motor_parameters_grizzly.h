/**
  * @file    pmsm_motor_parameters_grizzly.h
  * @brief   This file contains the parameters needed in order to configure the motor of grizzly motor.
  *
*/

/* Define to prevent recursive inclusion ---
----------------------------------*/
#ifndef __PMSM_MOTOR_PARAMETERS_GRIZZLY_H
#define __PMSM_MOTOR_PARAMETERS_GRIZZLY_H

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          10 /* Number of motor pole pairs */
#define RS                     0.071 /* Stator resistance , ohm*/
#define LS                     0.000150 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_TORQUE     1453  /* Nominal torque to apply to motor in cNm  */
#define STARTING_TORQUE    1718  /* Maximum starting torque to apply to motor in cNm  */
#define NOMINAL_PEAK_CURRENT    13000 /* Maximum current amplitude that can be injected
                                            per phase in digital Amps */
#define MOTOR_MAX_SPEED_RPM     1250 /*!< Maximum rated speed  */
#define MOTOR_MAGNET_FLUX       0.0175 
#define MOTOR_VOLTAGE_CONSTANT  22.5 /*!< Volts RMS ph-ph /kRPM */
#define ID_DEMAG                -1000 /*!< Demagnetization current */
#define MOTOR_MAX_TEMPERATURE_C 70    /* Maximum temperature in degree C */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!< Mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        60 /*!< Electrical phase shift in degree between the low to high
                                        transition of signal H1 and the zero crossing of the Bemf induced
                                        between phase A and B */

#endif
