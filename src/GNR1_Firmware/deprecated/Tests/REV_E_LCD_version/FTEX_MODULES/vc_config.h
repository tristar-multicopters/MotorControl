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
#include "host_comm.h"
#include "lcd_bafang_comm.h"
#include "lcd_eggrider_comm.h"


#define USE_MOTOR1												1
#define USE_MOTOR2												0

#define USE_SENSORLESS										0

#define DERATING_ENABLE										0
#define CANBUS_ENABLE											1
#define PAS_ENABLE												0
#define HOSTCOMM_ENABLE										0
#define ERROR_MANAGEMENT_ENABLE						0


extern VC_Handle_t VController;

extern SPI_Handle_t SPI0Manager;
extern MCP25625_Handle_t CANController;
extern RCM_Handle_t RegularConvertionManager;

#endif /* __VC_CONFIG_H */

