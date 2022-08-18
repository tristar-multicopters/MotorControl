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

#endif

#endif /*__VC_PARAMETERS_H*/

