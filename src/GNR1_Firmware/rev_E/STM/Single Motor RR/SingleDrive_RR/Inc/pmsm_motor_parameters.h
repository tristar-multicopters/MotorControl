/**
  ******************************************************************************
  * @file    pmsm_motor_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the motor to drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PMSM_MOTOR_PARAMETERS_H
#define __PMSM_MOTOR_PARAMETERS_H

#define VEHICLE_EBGO 					1
#define VEHICLE_GRIZZLY 			    2
#define VEHICLE_ECELLS                  3

/*______________________________________________________*/
/* Change parameter below to quickly configure firmware */
#define VEHICLE_SELECTION 		VEHICLE_GRIZZLY
#define VOLTAGE_OPENLOOP			0
#define POSITION_OPENLOOP			0
#define SWD_TORQUE_CONTROL		    0
/*______________________________________________________*/


#if VEHICLE_SELECTION == VEHICLE_GRIZZLY

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          10 /* Number of motor pole pairs */
#define RS                     0.071 /* Stator resistance , ohm*/
#define LS                     0.000150 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#ifdef Rev_F

#define NOMINAL_CURRENT         13000 /*!< Maximum current value */
#define NOMINAL_TORQUE          1453  /*!< Maximum Nominal torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */
#define STARTING_TORQUE         1718  /*!< Maximum Starting torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */     


#else

#define NOMINAL_CURRENT         14000 /*!< Maximum current value */
#define NOMINAL_TORQUE          1400 /*!< Maximum torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */
#define STARTING_TORQUE         1718  /*!< Maximum Starting torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */  

#endif

#define MOTOR_MAX_SPEED_RPM     1325 /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT  22.5 /*!< Volts RMS ph-ph /kRPM */
#define MOTOR_MAGNET_FLUX       0.0175  /* Value of motor flux, derived from tests*/
#define ID_DEMAG                -5000 /*!< Demagnetization current */
#define MOTOR_MAX_TEMPERATURE_C 70    /* Maximum temperature in degree C */
#define MOTOR_MAX_POWER         1200     /* Max power in Watt */
#define DYMANIC_TORQUE_END_SPEED  100   /* Speed beyond which Maximum torque is nominal torque */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        60 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR             400  /*!< Number of pulses per
                                            revolution */

#elif VEHICLE_SELECTION == VEHICLE_EBGO

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          23 /* Number of motor pole pairs */
#define RS                     0.50 /* Stator resistance , ohm*/
#define LS                     0.000200 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#ifdef Rev_F

#define NOMINAL_CURRENT         14000 /*!< Maximum current value */
#define NOMINAL_TORQUE          1400 /*!< Maximum torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */
#define STARTING_TORQUE         1718  /*!< Maximum Starting torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */  

#else

#define NOMINAL_CURRENT         14000 /*!< Maximum current value */
#define NOMINAL_TORQUE          1400 /*!< Maximum torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */
#define STARTING_TORQUE         1718  /*!< Maximum Starting torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */  

#endif

#define MOTOR_MAX_SPEED_RPM     2750 /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT  15.0 /*!< Volts RMS ph-ph /kRPM */
#define MOTOR_MAGNET_FLUX       0.02  /* Value of motor flux, derived from tests*/
#define ID_DEMAG                -5000 /*!< Demagnetization current */

#define MOTOR_MAX_TEMPERATURE_C 70    /* Maximum temperature in degree C */
#define MOTOR_MAX_POWER         1200     /* Max power in Watt */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        60 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR             400  /*!< Number of pulses per
                                            revolution */

#elif VEHICLE_SELECTION == VEHICLE_ECELLS

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          23 /* Number of motor pole pairs */
#define RS                     0.50 /* Stator resistance , ohm*/
#define LS                     0.000200 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/
#ifdef Rev_F

#define NOMINAL_CURRENT         14000 /*!< Maximum current value */
#define NOMINAL_TORQUE          1400 /*!< Maximum torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */
#define STARTING_TORQUE         1718  /*!< Maximum Starting torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */  

#else

#define NOMINAL_CURRENT         14000 /*!< Maximum current value */
#define NOMINAL_TORQUE          1400 /*!< Maximum torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */
#define STARTING_TORQUE         1718  /*!< Maximum Starting torque value in cNm, it is motor torque to obtain hub torque multiply it with gear train ratio. */  

#endif

#define MOTOR_MAX_SPEED_RPM     1500 /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT  15.0 /*!< Volts RMS ph-ph /kRPM */
#define MOTOR_MAGNET_FLUX       0.013  /* Value of motor flux, derived from tests*/
#define ID_DEMAG                -5000 /*!< Demagnetization current */

#define MOTOR_MAX_TEMPERATURE_C 70    /* Maximum temperature in degree C */
#define MOTOR_MAX_POWER         1500     /* Max power in Watt */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        60 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR             400  /*!< Number of pulses per
                                            revolution */

#endif

#endif /*__PMSM_MOTOR_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
