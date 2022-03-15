/**
  ******************************************************************************
  * @file    regular_conversion_manager.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles regular analog conversions
  *
	******************************************************************************
*/

	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REGULAR_CONVERSION_MANAGER_H
#define __REGULAR_CONVERSION_MANAGER_H

#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"


#define MAX_NUMBER_OF_CONVERSION 8
#define TASK_RCM_SAMPLE_TIME_TICK 5

//#define RCM_DEBUG

#ifdef RCM_DEBUG
#include "bsp.h"
#endif


typedef struct
{
	nrf_drv_timer_t* 							pTimerInstance;
	uint32_t 											samplingTime_ms;
	
	nrf_saadc_channel_config_t 		rcm_conversion_array[MAX_NUMBER_OF_CONVERSION];
	uint8_t 											rcm_nb_of_conversion;
	nrf_saadc_value_t 						rcm_buffer[2][MAX_NUMBER_OF_CONVERSION];
	uint8_t												bActiveBuffer;
	nrf_ppi_channel_t							ppi_channel;
	bool													isError;
	
} RCM_Handle_t;


uint8_t RCM_AddConv(RCM_Handle_t* pHandle, nrf_saadc_channel_config_t channel_config);

void RCM_RemoveConv(RCM_Handle_t* pHandle, uint8_t conv_handle);

int16_t RCM_ReadConv(RCM_Handle_t* pHandle, uint8_t conv_handle);

void RCM_Init(RCM_Handle_t* pHandle);





#endif /*__REGULAR_CONVERSION_MANAGER_H*/

