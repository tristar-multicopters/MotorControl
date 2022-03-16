/**
  ******************************************************************************
  * @file    throttle.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles throttle management
  *
	******************************************************************************
	*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __THROTTLE_H
#define __THROTTLE_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "bsp.h"
#include "app_button.h"
#include "app_uart.h"
#include "md_code_reg.h"
#include "uart_frame_communication_protocol.h"
#include "nrfx_timer.h"

#include "nrf_drv_saadc.h"


#define THROTTLE_TASK_SAMPLE_TIME_TICK		10


/**
  * @brief Throttle_Handle_t structure used for throttle monitoring
  *
  */
typedef struct
{          
	nrf_saadc_input_t hAnalogInput;			/**< It contains analog input number used for throttle sensing. */
  
  nrf_saadc_value_t hInstThrottle_d;  /**< It contains latest available instanteanous throttle value
																			This parameter is expressed in nrf_saadc_value_t */
	
  uint16_t hAvThrottle_d;          		/**< It contains latest available average throttle value.
																			This parameter is expressed in u16 */
	
  uint16_t hLowPassFilterBW;   				/**< used to configure the first order software filter bandwidth. */                 

} Throttle_Handle_t;


void TSK_THROTTLEacquire (void * pvParameter);


#endif /*__THROTTLE_H*/

