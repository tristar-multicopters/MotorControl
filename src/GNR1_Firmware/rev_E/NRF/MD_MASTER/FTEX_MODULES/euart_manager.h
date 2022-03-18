/**
  ******************************************************************************
  * @file    euart_manager.c
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   Low level module used for managing the communication over UART0 
	*					 (used normally for LCD displays).
	******************************************************************************
	*/
	
#ifndef __EUART_MANAGER_H
#define __EUART_MANAGER_H

#include "nrf_uart.h"
#include "nrf_drv_uart.h"
#include <cmsis_os2.h>

 /*************** TYPE DEFINITIONS ******************/ 
#define EUART_BUFFER_SIZE 8
#define EUART_FLAG			  0x10
// Protocol types
typedef enum
{
	EUART_HOST,
	EUART_BAFANG,
	EUART_FTEX,
	EUART_APT
}eUART_protocol_t;

typedef enum
{
    EUART_BYTE_RECEIVED,          // An event indicating a byte has been received
    EUART_BYTE_SENT,            	// An event indicating a byte has been sent
} eUART_evt_type_t;

typedef struct
{
    eUART_evt_type_t evt_type;   // Type of event
		uint8_t byte_to_send[1];   // Byte to send over the uart event
} eUART_evt_t;

typedef void (* eUART_event_handler_t) (eUART_evt_t * p_euart_event); // callback for LCD protocols

typedef struct
{
	nrf_drv_uart_t* p_uart_inst;         // uart instance
	eUART_protocol_t dev_type;				   // type of display or other device
	eUART_event_handler_t p_evt_handler; // callback to execute in the uart event handler
	uint8_t rx_byte[1];   						   // Received byte 
	uint8_t hError;											 // Code for managing errors
}eUART_handler_t;

/********************** FUNCTIONS **********************/

/* Function for initializing the uart0 instance */
void eUART_Init(eUART_handler_t * pHandle, nrf_drv_uart_config_t uart_config);

/* Function for sending the uart0 instance */
void eUART_Send(eUART_handler_t * pHandle, uint8_t * tx_buffer, uint8_t size);

/* Function for initializing the uart0 instance */
void eUART_Receive(eUART_handler_t * pHandle, uint8_t * rx_buffer);

#endif
