/**
  ******************************************************************************
  * @file    throttle.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles throttle management
  *
	******************************************************************************
*/

#include "throttle.h"


Throttle_Handle_t throttle_handle =
{
  .hAnalogInput        		 = NRF_SAADC_INPUT_AIN1,
	.hLowPassFilterBW				 = 20
};
Throttle_Handle_t * p_throttle_handle = &throttle_handle;

TaskHandle_t TSK_THROTTLEacquire_handle;
int16_t SAADC_Th_value;

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	BaseType_t yield_req = pdFALSE;

	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		vTaskNotifyGiveFromISR(TSK_THROTTLEacquire_handle, &yield_req);
		portYIELD_FROM_ISR(yield_req);
	}
}
	
static void Throttle_Init( Throttle_Handle_t * pHandle )
{
	nrf_saadc_channel_config_t channel_config =
			NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(pHandle->hAnalogInput);

	nrf_drv_saadc_init(NULL, saadc_callback);

	nrf_drv_saadc_channel_init(0, &channel_config);
	
	nrf_saadc_value_t res = 0;
	nrf_drv_saadc_buffer_convert(&res, 1);
	nrf_drv_saadc_calibrate_offset();
	if (! xTaskNotifyWait(0, 0, NULL, 50))
	{
		//TODO: No response from event handler
	}
}

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion. Mustt be called periodically.
  *
  *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
  */
static void Throttle_CalcAvValue( Throttle_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;

	hAux = pHandle->hInstThrottle_d;

	if ( hAux != 0xFFFFu )
	{
		wtemp =  ( uint32_t )( pHandle->hLowPassFilterBW ) - 1u;
		wtemp *= ( uint32_t ) ( pHandle->hAvThrottle_d );
		wtemp += hAux;
		wtemp /= ( uint32_t )( pHandle->hLowPassFilterBW );

		pHandle->hAvThrottle_d = ( uint16_t ) wtemp;
	}
}

/**@brief Task function to acquire throttle signal from ADC
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
void TSK_THROTTLEacquire (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	Throttle_Init(p_throttle_handle);

	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	while (true)
	{	
		nrf_drv_saadc_buffer_convert(&p_throttle_handle->hInstThrottle_d, 1);
		SAADC_Th_value = p_throttle_handle->hInstThrottle_d;
		nrf_drv_saadc_sample();
		
		vTaskDelayUntil( &xLastWakeTime, THROTTLE_TASK_SAMPLE_TIME_TICK );
		
		Throttle_CalcAvValue(p_throttle_handle);
	}
}

