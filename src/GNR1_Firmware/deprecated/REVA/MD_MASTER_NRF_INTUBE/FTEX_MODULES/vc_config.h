/**
  ******************************************************************************
  * @file    vc_config.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module declares global structures used by other vehicule control modules
  *
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VC_CONFIG_H
#define __VC_CONFIG_H

#include "md_comm.h"
#include "throttle.h"
#include "flashconfig.h"


extern Throttle_Handle_t ThrottleSensor;


#endif /* __VC_CONFIG_H */

