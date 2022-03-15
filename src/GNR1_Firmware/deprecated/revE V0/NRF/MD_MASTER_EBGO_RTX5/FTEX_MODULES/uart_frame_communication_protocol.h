/**
  ******************************************************************************
  * @file    uart_frame_communication_protocol.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module provides uart-specific structures and functions of
	*					 a frame based serial communication protocol. Adapted from STM32 MC SDK.
  *
	******************************************************************************
	*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UFCP_H
#define __UFCP_H

/* Includes ------------------------------------------------------------------*/
#include "frame_communication_protocol.h"
#include "nrf_uart.h"
#include "nrf_drv_uart.h"
#include "bsp.h"


 /**
  * @brief Error Code for an Overrun FCP message frame.
  *
  * Such a payload is sent to the remote when an USART overrun error occurs.
  *
  * The value of the error code is set by the Frame Communication Protocol and is taken in
  * a value space that is shared with the Motor Control Protocol. It thus cannot be
  * changed.
  */
#define UFCP_MSG_OVERRUN 0x08

 /* Exported types ------------------------------------------------------------*/
 
typedef enum
{
    UFCP_FRAME_RECEIVED,          /**< An event indicating that a frame has been received. */
    UFCP_FRAME_SENT,            /**< An event indicating that FCP has completed transmission of a frame. */
    UFCP_CRC_ERROR,            /**< An event indicating that FCP has received a frame but didn't pass CRC check. */
} ufcp_evt_type_t;

typedef struct
{
    ufcp_evt_type_t evt_type; /**< Type of event. */
		FCP_Frame_t frame;
} ufcp_evt_t;

typedef void (* ufcp_event_handler_t) (ufcp_evt_t * p_ufcp_event);
 

typedef struct {
  FCP_Handle_t _Super;
	
	nrf_drv_uart_t* p_uart_inst;
	ufcp_event_handler_t p_evt_handler;
	
	uint8_t rx_buffer[1];
	
} UFCP_Handle_t;



/* Exported functions ------------------------------------------------------- */

void UFCP_Init( UFCP_Handle_t * pHandle, nrf_drv_uart_config_t uart_config, ufcp_event_handler_t evt_handler);

uint8_t UFCP_Receive(UFCP_Handle_t * pHandle);

uint8_t UFCP_Send(UFCP_Handle_t * pHandle, uint8_t code, uint8_t *buffer, uint8_t size);

void UFCP_AbortReceive(UFCP_Handle_t * pHandle);

void * UFCP_RX_IRQ_Handler(UFCP_Handle_t * pHandle, unsigned short rx_data );

void UFCP_TX_IRQ_Handler(UFCP_Handle_t * pHandle);

void UFCP_OVR_IRQ_Handler(UFCP_Handle_t * pHandle);

void UFCP_TIMEOUT_IRQ_Handler(UFCP_Handle_t * pHandle);

void uart_ufcp_event_handler(nrf_drv_uart_event_t * p_event, void* p_context);

#endif /* __UFCP_H */
