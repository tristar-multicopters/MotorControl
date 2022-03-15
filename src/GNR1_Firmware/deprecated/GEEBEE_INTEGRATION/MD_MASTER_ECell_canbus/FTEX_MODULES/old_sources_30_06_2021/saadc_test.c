#include "saadc_test.h"

extern TaskHandle_t m_voltMeas_thread;
md_reg_t host_REG; // Register to test

// Handle the events once the samples are received in the buffer
void saadc_callback_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if(p_event -> type == NRFX_SAADC_EVT_DONE) // check if the sampling is done and we are ready to take these samples for processing
    {
      ret_code_t err_code; // a variable to hold errors code

// A function to take the samples (which are in the buffer in the form of 2's complement), and convert them to 16-bit interger values
      err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLE_BUFFER_LEN);
      APP_ERROR_CHECK(err_code); // check for errors

// For loop is used to read and process each variable in the buffer, if the buffer size is 1, we don't need for loop
//      for(int i = 0; i < SAMPLE_BUFFER_LEN; i++)
//      {
				// Perform some calculations to convert this value back to the voltage
        host_REG.bus_voltage_mes = (p_event->data.done.p_buffer[0])*(3600.0/4096.0);	
//      }
    }
}

// Create a function to initialize the saadc 
void saadc_init(void)
{
  ret_code_t err_code; // variable to store the error code

// Create a struct to hold the default configurations which will be used to initialize the adc module.
// make sure to use the right pins
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

// Initialize the adc module Null is for configurations, they can be configured via CMSIS Configuration wizard so we don't need to pass anything here
  err_code = nrf_drv_saadc_init(NULL, saadc_callback_handler);
  APP_ERROR_CHECK(err_code);

// allocate the channel along with the configurations
  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

// allocate first buffer where the values will be stored once sampled
  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLE_BUFFER_LEN);
  APP_ERROR_CHECK(err_code);

// allocate second buffer where the values will be stored if overwritten on first before reading
  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLE_BUFFER_LEN);
  APP_ERROR_CHECK(err_code);
}

void saadc_init_Task(void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	saadc_init();
	
	while(true)
	{	
		nrfx_saadc_sample();
		vTaskDelay(50);
	}
}
