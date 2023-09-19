/**
  * @file    drive_parameters.h
  * @brief   This file contains the parameters needed in order to configure the motor to drive.
  *
*/

/* Define to prevent recursive inclusion ---
----------------------------------*/
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

#include "gnr_parameters.h"


#if VEHICLE_SELECTION == VEHICLE_E_CELLS

#include "drive_parameters_e_cells.h"

#elif VEHICLE_SELECTION == VEHICLE_R48_750W

#include "drive_parameters_r48_750w.h"

#elif VEHICLE_SELECTION == VEHICLE_TSUGAWA

#include "drive_parameters_tsugawa.h"

#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#include "drive_parameters_nidec.h"

#elif VEHICLE_SELECTION == VEHICLE_QUIETKAT

#include "drive_parameters_quietkat.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC_CITI_500W

#include "drive_parameters_velec_citi_500w.h"

#elif VEHICLE_SELECTION == VEHICLE_A2_350W

#include "drive_parameters_a2_350w.h"

#elif VEHICLE_SELECTION == VEHICLE_UTK_350W 

#include "drive_parameters_utk_350w.h"

#elif VEHICLE_SELECTION == VEHICLE_A2_500W 

#include "drive_parameters_a2_500w.h"

#endif

/****** PHASE WIRE DISCONNECTIION DETECTIOT *******/
#define CURRENT_AVG_WIN_SIZE                32          // the moving average window size
#define PHASE_WIRE_DISCONNECT_WAIT_MCCYCLE  100         // The time to wait and check before raising warning

/******** STUCK PROTECTION SETTING SECTION ********/

#define STUCK_TIMER_MAX_TICKS               2000                                                                // protection timeout in MC Layer ticks  
#define STUCK_TIMER_MAX_COUNTS              STUCK_TIMER_MAX_TICKS * SPEED_LOOP_FREQUENCY_HZ/1000u - 1u          // protection timeout
#define STUCK_MIN_TORQUE                    200                                                                 // minimum torque that can cause the protection to get activated
#define STUCK_LOW_VOLTAGE_THRESHOLD         42                                                                  // this parameter is used to reduce protection timeout when battery SoC is low
#define STUCK_TIMER_MAX_COUNTS_LOWBATTERY   (STUCK_TIMER_MAX_TICKS/10) * SPEED_LOOP_FREQUENCY_HZ/1000u - 1u     // the protection timeout battery SoC is detected as low

/******** SPEED CONTROL SETTING SECTION ********/
#define SPDCTRL_UPPER_INTEGRAL_LIMIT                2097152 // 2^21     // The maximum allowed value for Integral Term of Speed Control PID

#define VIBRATION_PATTERN                   0xAAAA   // = 0b1010101010101010 which is 8 time of direction change
#endif /*__DRIVE_PARAMETERS_H*/
