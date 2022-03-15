/**
  ******************************************************************************
  * @file    comm_usbd.c
  * @author  Jorge Andres Polo, FTEX
  * @brief   This module provides serial communication with the host by using 
	*					 the USBD CDC ACM module
  *
	******************************************************************************
	*/
	
#include "usbd_comm.h"
#include "queue.h"

TaskHandle_t TSK_usbd_handle;
extern TaskHandle_t TSK_ReplyToHost_handle;

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm_polo,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
						//bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
						//bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

static void usb_new_event_isr_handler(app_usbd_internal_evt_t const * const p_event, bool queued)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    UNUSED_PARAMETER(p_event);
    UNUSED_PARAMETER(queued);
    ASSERT(TSK_usbd_handle != NULL);
    /* Release the semaphore */
    vTaskNotifyGiveFromISR(TSK_usbd_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm_polo,
                                                   m_rx_buffer,
                                                   RX_BUFFER_SIZE);
						UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
							/*Reset Rx buffer and index buffer */
							resetRxBuffer();
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {		
						BaseType_t yield_request;
            ret_code_t ret;
            do
            {
							/*Build a frame with received data*/
							m_rx_data[m_buffer_index++] = m_rx_buffer[0];
							ret = app_usbd_cdc_acm_read(&m_app_cdc_acm_polo,
																					 m_rx_buffer,
																					 RX_BUFFER_SIZE);             
            } while (ret == NRF_SUCCESS);
												
						/*Build the buffer to keep data on local registers*/
						// After this, execute Host sending task...
						vTaskNotifyGiveFromISR(TSK_ReplyToHost_handle, &yield_request);
						portYIELD_FROM_ISR(yield_request);
						
            break;
        }
        default:
            break;
    }
}

uint8_t *getRxBuffer(void)
{
	return m_rx_data;
}

static void resetRxBuffer(void)
{
  memset(m_rx_data,0,sizeof(m_rx_data));
	m_buffer_index = 0;
}

void USBD_send_data(uint8_t *dataRx, uint16_t length)
{
	ret_code_t ret;
	ret = app_usbd_cdc_acm_write(&m_app_cdc_acm_polo, dataRx, length);
	
	if(ret != NRF_SUCCESS)
		return;
}

void TSK_init_usbd(void * arg)
{
		ret_code_t ret;
	
		/* Clock initialisation for USBD comm*/
		nrf_drv_clock_init();
	
		static const app_usbd_config_t usbd_config = {
					.ev_isr_handler = usb_new_event_isr_handler,
					.ev_state_proc = usbd_user_ev_handler};
		UNUSED_PARAMETER(arg);
									
		app_usbd_serial_num_generate();
				
		ret = app_usbd_init(&usbd_config);
		APP_ERROR_CHECK(ret);
				
		app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm_polo);
		ret = app_usbd_class_append(class_cdc_acm);
		APP_ERROR_CHECK(ret);
		
		ret = app_usbd_power_events_enable();
		APP_ERROR_CHECK(ret);
					
		// Set the first event to make sure that USB queue is processed after it is started
    UNUSED_RETURN_VALUE(xTaskNotifyGive(xTaskGetCurrentTaskHandle()));
					
		for(;;)
		{
			/* Waiting for event */
      UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, USB_THREAD_MAX_BLOCK_TIME)); // Jorge: Problematic...
			while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
		}
}
