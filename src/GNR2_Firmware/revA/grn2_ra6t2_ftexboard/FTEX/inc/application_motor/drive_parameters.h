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

#elif VEHICLE_SELECTION == VEHICLE_E_CELLS

#include "drive_parameters_e_cells.h"

#elif VEHICLE_SELECTION == VEHICLE_R48_750W

#include "drive_parameters_r48_750w.h"

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

#elif VEHICLE_SELECTION == VEHICLE_VELEC_CITI_500W

#include "drive_parameters_velec_citi_500w.h"

#elif VEHICLE_SELECTION == VEHICLE_A2_350W

#include "drive_parameters_a2_350w.h"

#elif VEHICLE_SELECTION == VEHICLE_UTK_350W 

#include "drive_parameters_utk_350w.h"

#elif VEHICLE_SELECTION == VEHICLE_A2_500W 

#include "drive_parameters_a2_500w.h"

#endif

#endif /*__DRIVE_PARAMETERS_H*/
