/**
  ******************************************************************************
  * @file    wheel_speed_sensor.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles wheel speed computation from sensor external sensor
  *
	******************************************************************************
	*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WHEEL_SPEED_SENSOR_H
#define __WHEEL_SPEED_SENSOR_H

#include "stdlib.h"
#include "stdint.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "regular_conversion_manager.h"
#include "speed_pulse_read.h"



typedef struct {
	
	SPR_Handle_t * wSpulse;				/* Pointer to torque handle */
	
} WAS_Handle_t;


void WSS_Init(WAS_Handle_t* pHandle);
int32_t WSS_GetSpeed(WAS_Handle_t* pHandle);
int16_t WSS_GetDirection(WAS_Handle_t* pHandle);

#endif /*__WHEEL_SPEED_SENSOR_H*/

