/**
  ******************************************************************************
  * @file    pedal_assist.c
  * @author  Ronak Nemade, FTEX
  * @brief   This file defines the functions used in higher level modules for pedal assist
  *
	******************************************************************************
*/


#include "pedal_assist.h"

static PAS_Handle_t* m_pPAS_Handle;


void timer2_compare0_handler(nrf_timer_event_t event_type, void * p_context)
{
	// This handler is called when there is a timeout on the timer module
		m_pPAS_Handle->count = 0;
		m_pPAS_Handle->pas_state = false;

};

void encoder_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
// This handler is called when there is a toggle event on encoder pin
	
	 if (m_pPAS_Handle->count < 1)
	 {
		 m_pPAS_Handle->count ++;
	 }
	 else
	 {

			if(!m_pPAS_Handle->pas_state && m_pPAS_Handle->pas_level<=m_pPAS_Handle->hNumberOfLevel)
			{
			
				m_pPAS_Handle->hTref = ((m_pPAS_Handle->hIqmax*m_pPAS_Handle->pas_level)/m_pPAS_Handle->hNumberOfLevel);
						
				m_pPAS_Handle->pas_state = true;
				
			}
			else if (m_pPAS_Handle->pas_state && m_pPAS_Handle->pas_level<=m_pPAS_Handle->hNumberOfLevel)
			{
			
				m_pPAS_Handle->hTref = ((m_pPAS_Handle->hIqmax*m_pPAS_Handle->pas_level)/m_pPAS_Handle->hNumberOfLevel);
			
			}
	}
				
};



void PAS_Init(PAS_Handle_t* pHandle)
{
	m_pPAS_Handle = pHandle;
	nrf_ppi_channel_t ppi_channel;
	uint32_t gpiote_toggle_evt_addr;
  uint32_t timer_task_addr;

	m_pPAS_Handle->pas_state = false;
	m_pPAS_Handle->pas_level = PAS_LEVEL_1;
	
	switch (m_pPAS_Handle->pas_config.pas_type)
	{
		case PAS_TORQUE:
			{
			// configure input pin to execute encoder handle
			nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
			nrfx_gpiote_in_init(m_pPAS_Handle->pas_config.pas_speed_encoder_pin,&config,&encoder_handler);
			nrfx_gpiote_in_event_enable(m_pPAS_Handle->pas_config.pas_speed_encoder_pin,true);
				
			// configure timer 2 to operate in timeout mode
			nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
			timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
			
			nrf_drv_timer_init(m_pPAS_Handle->pas_config.pTimerInstance, &timer_cfg, timer2_compare0_handler);				
			uint32_t ticks = nrf_drv_timer_ms_to_ticks(m_pPAS_Handle->pas_config.pTimerInstance, m_pPAS_Handle->pas_config.pas_timer_timeout_ms);
			nrf_drv_timer_extended_compare(m_pPAS_Handle->pas_config.pTimerInstance,
																 NRF_TIMER_CC_CHANNEL0,
																 ticks,
																 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
																 true);
				
			// Assign PPI to clear timer from GPIO input	
				
			nrf_drv_ppi_channel_alloc(&ppi_channel);
			gpiote_toggle_evt_addr = nrf_drv_gpiote_in_event_addr_get(m_pPAS_Handle->pas_config.pas_speed_encoder_pin);
			timer_task_addr = nrf_drv_timer_task_address_get(m_pPAS_Handle->pas_config.pTimerInstance,NRF_TIMER_TASK_CLEAR); 
			
			nrf_drv_ppi_channel_assign(ppi_channel, gpiote_toggle_evt_addr, timer_task_addr);
			
			nrf_drv_ppi_channel_enable(ppi_channel);
			
			nrf_drv_timer_enable(m_pPAS_Handle->pas_config.pTimerInstance);
				
		break;
			}
		case PAS_SINCOS:
			{
			
		break;		
			}	
		case PAS_SIN:
			{
			
		break;		
			}			
	}
	
}

void PAS_SetLevel(PAS_Handle_t* pHandle, pas_level_t Level)
{
	pHandle->pas_level = Level;
}


int16_t PAS_CalcIqref(PAS_Handle_t* pHandle,int16_t Tref)
{
	if(Tref == 0 && pHandle->pas_state == true)
	{
			return pHandle->hTref;
	
	}
	else
	{return Tref;}
	
}
