/**
  ******************************************************************************
  * @file    comm_usbd.h
  * @author  Jorge Andres Polo, FTEX
  * @brief   This module provides serial communication with the host by using 
	*					 the USBD CDC ACM module
  *
	******************************************************************************
	*/
#ifndef __USBD_COMM_H
#define __USBD_COMM_H

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_serial_num.h"
#include "app_usbd_cdc_acm.h"

#include "bsp_cli.h"
#include "nrf_cli.h"
//#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "frame_communication_protocol.h"
#include "mc_defines.h"
#include <cmsis_os2.h>

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

//#define READ_SIZE 1
#define RX_BUFFER_SIZE 1

/**
 * The maximum delay inside the USB task to wait for an event.
 */
#define USB_THREAD_MAX_BLOCK_TIME portMAX_DELAY

#define SPEED_TEST 0x12345678
#define IQ_TEST 0x9ABC

//static FCP_Frame_t frame_RX;
//static FCP_Frame_t frame_TX;
static uint8_t m_rx_buffer[RX_BUFFER_SIZE];
static uint8_t m_rx_data[FCP_MAX_PAYLOAD_SIZE];
static uint8_t m_buffer_index = 0;


/**************** Prototype Functions ***************/
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

/*Function for reseting Rx buffer and index buffer */
static void resetRxBuffer(void);

uint8_t *getRxBuffer(void);

void USBD_send_data(uint8_t *dataRx, uint16_t length);

void TSK_init_usbd(void * arg);

#endif

