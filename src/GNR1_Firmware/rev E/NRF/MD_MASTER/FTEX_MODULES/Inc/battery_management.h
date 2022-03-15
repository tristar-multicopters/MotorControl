/**
  ******************************************************************************
  * @file    battery_management.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles battery related algorithms
  *
	******************************************************************************
	*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BATTERY_MANAGEMENT_H
#define __BATTERY_MANAGEMENT_H

#include "md_interface.h"

typedef enum
{
	LIION,
	LIPO,
} Battery_Type_h;

typedef struct
{
	MDI_Handle_t pMDI;
	uint8_t NumberOfCell_S;
	uint8_t NumberOfCell_P;
	Battery_Type_h BatteryType;
	
} Battery_Handle_t;


void BAT_Init(Battery_Handle_t * pHandle);

uint16_t BAT_ComputeSOC(Battery_Handle_t * pHandle);



#endif /*__BATTERY_MANAGEMENT_H*/

