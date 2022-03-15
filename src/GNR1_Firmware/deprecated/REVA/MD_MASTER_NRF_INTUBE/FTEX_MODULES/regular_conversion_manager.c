/**
  ******************************************************************************
  * @file    regular_conversion_manager.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles regular analog conversions
  *
	******************************************************************************
*/

#include "regular_conversion_manager.h"


nrf_saadc_channel_config_t rcm_conversion_array[MAX_NUMBER_OF_CONVERSION];
uint8_t rcm_nb_of_conversion = 0;
nrf_saadc_value_t rcm_buffer[MAX_NUMBER_OF_CONVERSION];


static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		nrf_drv_saadc_buffer_convert(rcm_buffer, rcm_nb_of_conversion);
	}
}

uint8_t RCM_AddConv(nrf_saadc_channel_config_t channel_config)
{
	uint8_t handle = rcm_nb_of_conversion;
	rcm_conversion_array[handle] = channel_config;
	rcm_nb_of_conversion += 1;
	
	return handle;
}

void RCM_RemoveConv(uint8_t handle)
{
	for (uint8_t i = 0; i < rcm_nb_of_conversion-handle; i++)
	{
		if (handle + i + 1 < MAX_NUMBER_OF_CONVERSION) //Check if last 
		{
			rcm_conversion_array[handle + i] = rcm_conversion_array[handle + i + 1];
		}
		else
		{
			//Nothing to do
		}
	}
	rcm_nb_of_conversion -= 1;
}

int16_t RCM_ReadConv(uint8_t handle)
{
	return (int16_t)rcm_buffer[handle] << 4;
}

void RCM_Init(void)
{
	nrf_drv_saadc_config_t saadc_config = 
	{
		.resolution = NRF_SAADC_RESOLUTION_12BIT,
		.oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
		.interrupt_priority = 5,
		.low_power_mode = false,
	};
	
	nrf_drv_saadc_init(&saadc_config, saadc_callback);
	
	for (uint8_t i = 0; i < rcm_nb_of_conversion; i++)
	{
		nrf_drv_saadc_channel_init(0, &(rcm_conversion_array[i]));
	}
	
	nrf_drv_saadc_buffer_convert(rcm_buffer, rcm_nb_of_conversion);
}

void TSK_RCM (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	RCM_Init();

	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	while (true)
	{	
		nrf_drv_saadc_sample();
		
		vTaskDelayUntil( &xLastWakeTime, TASK_RCM_SAMPLE_TIME_TICK );
	}
}
