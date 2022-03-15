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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "nrfx_timer.h"
#include "nrf_drv_saadc.h"


#define MAX_NUMBER_OF_CONVERSION 8
#define TASK_RCM_SAMPLE_TIME_TICK 5

extern TaskHandle_t TSK_RCM_handle;


uint8_t RCM_AddConv(nrf_saadc_channel_config_t channel_config);

void RCM_RemoveConv(uint8_t handle);

int16_t RCM_ReadConv(uint8_t handle);

void RCM_Init(void);

	/**@brief Task function to read ADCs at a fixed time step*/
void TSK_RCM (void * pvParameter);



#endif /*__REGULAR_CONVERSION_MANAGER_H*/

