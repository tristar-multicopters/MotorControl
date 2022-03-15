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

#include "board_hardware.h"
#include "vc_interface.h"
#include "canbus_management.h"

extern VC_Handle_t VControl;
extern MD_Handle_t MotorDrive1;
extern MD_Handle_t MotorDrive2;


#endif /* __VC_CONFIG_H */

