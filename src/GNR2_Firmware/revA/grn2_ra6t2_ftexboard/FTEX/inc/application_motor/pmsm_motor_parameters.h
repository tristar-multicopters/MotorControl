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

#endif

#endif /*__PMSM_MOTOR_PARAMETERS_H*/

