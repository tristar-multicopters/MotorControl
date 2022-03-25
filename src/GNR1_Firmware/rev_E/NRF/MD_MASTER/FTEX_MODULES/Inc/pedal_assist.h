/**
  ******************************************************************************
  * @file    pedal_assist.h
  * @author  Ronak Nemade, FTEX
  * @brief   This file defines the handles, constantas and function prototypes used in higher level modules for pedal assist
  *
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PEDALASSIST_H
#define __PEDALASSIST_H

#include "stdlib.h"
#include "stdint.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "regular_conversion_manager.h"


typedef struct {
	
	nrf_drv_timer_t* 	pTimerInstance;
	
	nrf_saadc_channel_config_t hChannelConfig;		/**< It contains analog channel configuration used for torque sensing. */
  uint8_t convHandle;            								/*!< handle to the regular conversion */ 
	RCM_Handle_t* pRegularConversionManager;
	
	nrfx_gpiote_pin_t wSinPinNumber;
	nrfx_gpiote_pin_t wCosPinNumber;
	
} PAS_Config_t;


typedef struct {
	
	PAS_Config_t sConfig;
	
	uint8_t bLevel;
	uint8_t bMaxLevel;
	
} PAS_Handle_t;


void PAS_Init(PAS_Handle_t* pHandle);

int32_t PAS_GetSpeed(PAS_Handle_t* pHandle);

int16_t PAS_GetDirection(PAS_Handle_t* pHandle);

int16_t PAS_GetTorque(PAS_Handle_t* pHandle);



#endif
