/**
  * @file    pmsm_motor_parameters.h
  * @brief   This file contains the parameters needed in order to configure the motor to drive.
  *
*/

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __PMSM_MOTOR_PARAMETERS_H
#define __PMSM_MOTOR_PARAMETERS_H

#include "gnr_parameters.h"

#if VEHICLE_SELECTION == VEHICLE_GRIZZLY

#include "pmsm_motor_parameters_grizzly.h"

#elif VEHICLE_SELECTION == VEHICLE_EBGO

#include "pmsm_motor_parameters_ebgo.h"

#elif VEHICLE_SELECTION == VEHICLE_E_CELLS

#include "pmsm_motor_parameters_e_cells.h"

#elif VEHICLE_SELECTION == VEHICLE_APOLLO

#include "pmsm_motor_parameters_apollo.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC

#include "pmsm_motor_parameters_velec.h"

#elif VEHICLE_SELECTION == VEHICLE_WHEEL

#include "pmsm_motor_parameters_wheel.h"

#elif VEHICLE_SELECTION == VEHICLE_TSUGAWA

#include "pmsm_motor_parameters_tsugawa.h"

#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#include "pmsm_motor_parameters_nidec.h"

#elif VEHICLE_SELECTION == VEHICLE_QUIETKAT

#include "pmsm_motor_parameters_quietkat.h"

#elif VEHICLE_SELECTION == VEHICLE_RBS_MB

#include "pmsm_motor_parameters_rbs_mb.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC_CITI_500W

#include "pmsm_motor_parameters_velec_citi_500w.h"

#endif

#endif /*__PMSM_MOTOR_PARAMETERS_H*/

