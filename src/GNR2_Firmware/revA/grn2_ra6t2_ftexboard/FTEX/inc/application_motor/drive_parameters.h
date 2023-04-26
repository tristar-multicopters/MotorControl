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


#if VEHICLE_SELECTION == VEHICLE_GRIZZLY

#include "drive_parameters_grizzly.h"

#elif VEHICLE_SELECTION == VEHICLE_EBGO

#include "drive_parameters_ebgo.h"

#elif VEHICLE_SELECTION == VEHICLE_E_CELLS

#include "drive_parameters_e_cells.h"

#elif VEHICLE_SELECTION == VEHICLE_APOLLO

#include "drive_parameters_apollo.h"

#elif VEHICLE_SELECTION == VEHICLE_VELEC

#include "drive_parameters_velec.h"

#elif VEHICLE_SELECTION == VEHICLE_WHEEL

#include "drive_parameters_wheel.h"

#elif VEHICLE_SELECTION == VEHICLE_TSUGAWA

#include "drive_parameters_tsugawa.h"

#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#include "drive_parameters_nidec.h"

#elif VEHICLE_SELECTION == VEHICLE_QUIETKAT

#include "drive_parameters_quietkat.h"

#elif VEHICLE_SELECTION == VEHICLE_RBS_MB

#include "drive_parameters_rbs_mb.h"

#elif VEHICLE_SELECTION == VEHICLE_URBAN

#include "drive_parameters_urban.h"

#endif

#endif /*__DRIVE_PARAMETERS_H*/
