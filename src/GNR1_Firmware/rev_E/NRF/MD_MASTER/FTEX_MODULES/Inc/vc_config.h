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
#include "lcd_bafang_comm.h"
#include "lcd_ftex_comm.h"
#include "lcd_apt_comm.h"
#include "ev_config_tool.h"

#define VEHICLE_DEFAULT			0
#define VEHICLE_ECELL 			1
#define VEHICLE_EBGO 				2
/*
VEHICLE SELECTION
------------------------------------
Change this define based on vehicle application
------------------------------------
*/
#define VEHICLE_SELECTION 	VEHICLE_ECELL
/*------------------------------------*/


#define CANBUS_ENABLE	0


extern VCI_Handle_t VCInterfaceHandle;
extern SPI_Handle_t SPI0Manager;
extern MCP25625_Handle_t CANController;
extern RCM_Handle_t RegularConvertionManager;
extern eUART_protocol_t EUART_handle_t;

#endif /* __VC_CONFIG_H */

