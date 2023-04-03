/**
  * @file    vc_parameters.h
	* @author	 Sami Bouzid, FTEX inc	 
  * @brief   This file contains the parameters needed in order to configure vehicle control layer.
  *
*/

/* Define to prevent recursive inclusion --- 
----------------------------------*/
#ifndef __VC_PARAMETERS_H
#define __VC_PARAMETERS_H

#include "gnr_parameters.h"


#if VEHICLE_SELECTION == VEHICLE_GRIZZLY

#include "vc_parameters_grizzly.h"

#elif VEHICLE_SELECTION == VEHICLE_EBGO

#include "vc_parameters_ebgo.h"

#elif VEHICLE_SELECTION == VEHICLE_E_CELLS

#include "vc_parameters_e_cells.h"

#elif VEHICLE_SELECTION == VEHICLE_APOLLO

#include "vc_parameters_apollo.h"


#elif VEHICLE_SELECTION == VEHICLE_VELEC

#include "vc_parameters_velec.h"

#elif VEHICLE_SELECTION == VEHICLE_WHEEL

#include "vc_parameters_wheel.h"

#elif VEHICLE_SELECTION == VEHICLE_TSUGAWA

#include "vc_parameters_tsugawa.h"

#elif VEHICLE_SELECTION == VEHICLE_NIDEC

#include "vc_parameters_nidec.h"

#elif VEHICLE_SELECTION == VEHICLE_QUIETKAT

#include "vc_parameters_quietkat.h"

#endif

#endif /*__VC_PARAMETERS_H*/

