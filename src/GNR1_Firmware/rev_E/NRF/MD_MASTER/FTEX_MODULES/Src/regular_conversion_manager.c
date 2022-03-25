/**
  ******************************************************************************
  * @file    regular_conversion_manager.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles regular analog conversions
  *
	******************************************************************************
*/

#include "regular_conversion_manager.h"

static RCM_Handle_t* m_pRCM_Handle;

static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	#ifdef RCM_DEBUG
	bsp_board_led_invert(BSP_BOARD_LED_0);
	#endif
}

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		#ifdef RCM_DEBUG
		bsp_board_led_invert(BSP_BOARD_LED_1);
		#endif
		
		if ( p_event->data.done.p_buffer == &(m_pRCM_Handle->rcm_buffer[0][0]) )
		{
			m_pRCM_Handle->bActiveBuffer = 0;
		}
		else if ( p_event->data.done.p_buffer == &(m_pRCM_Handle->rcm_buffer[1][0]) )
		{
			m_pRCM_Handle->bActiveBuffer = 1;
		}
		else
		{
			m_pRCM_Handle->isError = true;
		}
				
		nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, m_pRCM_Handle->rcm_nb_of_conversion);
	}
}


uint8_t RCM_AddConv(RCM_Handle_t* pHandle, nrf_saadc_channel_config_t channel_config)
{
	uint8_t conv_handle = pHandle->rcm_nb_of_conversion;
	pHandle->rcm_conversion_array[conv_handle] = channel_config;
	pHandle->rcm_nb_of_conversion += 1;
	
	return conv_handle;
}

void RCM_RemoveConv(RCM_Handle_t* pHandle, uint8_t conv_handle)
{
	for (uint8_t i = 0; i < pHandle->rcm_nb_of_conversion-conv_handle; i++)
	{
		if (conv_handle + i + 1 < MAX_NUMBER_OF_CONVERSION) //Check if last 
		{
			pHandle->rcm_conversion_array[conv_handle + i] = pHandle->rcm_conversion_array[conv_handle + i + 1];
		}
		else
		{
			//Nothing to do
		}
	}
	pHandle->rcm_nb_of_conversion -= 1;
}

int16_t RCM_ReadConv(RCM_Handle_t* pHandle, uint8_t conv_handle)
{
	if(pHandle->rcm_buffer[m_pRCM_Handle->bActiveBuffer][conv_handle] < 0)
		pHandle->rcm_buffer[m_pRCM_Handle->bActiveBuffer][conv_handle] = 0;
	return (int16_t)pHandle->rcm_buffer[m_pRCM_Handle->bActiveBuffer][conv_handle] << 4;
}

void RCM_Init(RCM_Handle_t* pHandle)
{	
	#ifdef RCM_DEBUG
	bsp_board_init(BSP_INIT_LEDS);
	#endif
	
	m_pRCM_Handle = pHandle;
	m_pRCM_Handle->isError = false;
	
	/* SAADC Init */
	nrf_drv_saadc_config_t saadc_config = 
	{
		.resolution = NRF_SAADC_RESOLUTION_12BIT,
		.oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
		.interrupt_priority = 5,
		.low_power_mode = false,
	};
	nrf_drv_saadc_init(&saadc_config, saadc_callback);
	
	for (uint8_t i = 0; i < pHandle->rcm_nb_of_conversion; i++)
	{
		nrf_drv_saadc_channel_init(i, &(pHandle->rcm_conversion_array[i]));
	}
	
	/* Timer & PPI Init */
	nrf_drv_ppi_init();
	
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
	nrf_drv_timer_init(pHandle->pTimerInstance, &timer_cfg, timer_handler);
	uint32_t ticks = nrf_drv_timer_ms_to_ticks(pHandle->pTimerInstance, pHandle->samplingTime_ms);
	nrf_drv_timer_extended_compare(pHandle->pTimerInstance,
																 NRF_TIMER_CC_CHANNEL0,
																 ticks,
																 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
																 true);
	nrf_drv_timer_enable(pHandle->pTimerInstance);
	
	uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(pHandle->pTimerInstance,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();
	
	nrf_drv_ppi_channel_alloc(&pHandle->ppi_channel);
  nrf_drv_ppi_channel_assign(pHandle->ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
																					
	nrf_drv_ppi_channel_enable(pHandle->ppi_channel);
	
	nrf_drv_saadc_buffer_convert(m_pRCM_Handle->rcm_buffer[0], m_pRCM_Handle->rcm_nb_of_conversion);
	nrf_drv_saadc_buffer_convert(m_pRCM_Handle->rcm_buffer[1], m_pRCM_Handle->rcm_nb_of_conversion);
	
	//todo: handle errors
}


