/**
  * @file    pmsm_motor_parameters_velec.h
  * @brief   This file contains the parameters needed in order to configure the motor of velec motor.
  *
*/

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __PMSM_MOTOR_PARAMETERS_VELEC_H
#define __PMSM_MOTOR_PARAMETERS_VELEC_H

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          8 /* Number of motor pole pairs */
#define RS                     0.1 /* Stator resistance , ohm*/
#define LS                     0.00000235 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_TORQUE     1178  /* Nominal torque to apply to motor in cNm  */
#define STARTING_TORQUE    1300  /* Maximum starting torque to apply to motor in cNm  */
#define NOMINAL_PEAK_CURRENT    13000 /* Maximum current amplitude that can be injected
                                            per phase in digital Amps */
#define MOTOR_MAX_SPEED_RPM     2100   /*!< Maximum rated speed  */
                                       /* Old Example 2750 for 38Km/h */
#define MOTOR_MAGNET_FLUX       0.0195 /* Refers to the Flux of Permanent magnets used in the motor, derived by performing motor tests */
#define MOTOR_VOLTAGE_CONSTANT  28.84 /*!< Volts RMS ph-ph /kRPM */
#define ID_DEMAG                -1000 /*!< Demagnetization current */
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

