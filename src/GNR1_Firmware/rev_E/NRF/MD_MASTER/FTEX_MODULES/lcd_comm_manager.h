/**
  ******************************************************************************
  * @file    lcd_comm_manager.h
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   Low level module used for managing the communication with compatible
	*          LCD displays.
	*					 Available displays:
	*					 1. Host protocol
  *					 2. Bafang protocol
	*					 ...
	******************************************************************************
	*/
	
#ifndef __LCD_COMM_MANAGER_H
#define __LCD_COMM_MANAGER_H

#include "nrf_uart.h"
#include "nrf_drv_uart.h"

 /*************** TYPE DEFINITIONS ******************/ 

// Protocol types
typedef enum
{
	LCD_HOST,
	LCD_BAFANG,
}lcd_protocol_t;

typedef enum
{
    LCD_BYTE_RECEIVED,          // An event indicating a byte has been received
    LCD_BYTE_SENT,            	// An event indicating a byte has been sent
} lcd_evt_type_t;

typedef struct
{
    lcd_evt_type_t evt_type;   // Type of event
		uint8_t byte_to_send[1];   // Byte to send over the uart event
} lcd_evt_t;

typedef void (* lcd_event_handler_t) (lcd_evt_t * p_lcd_event); // callback for LCD protocols

typedef struct
{
	nrf_drv_uart_t* p_uart_inst;       // uart instance
	lcd_protocol_t lcd_type;					 // type of display
	lcd_event_handler_t p_evt_handler; // callback to execute in the uart event handler
	uint8_t rx_byte[1];   						 // Received byte 
}LCD_handler_t;

/********************** FUNCTIONS **********************/

/* Function for initializing the uart0 instance */
void LCD_Init(LCD_handler_t * pHandle, nrf_drv_uart_config_t uart_config);

/* Function for sending the uart0 instance */
void LCD_Send(LCD_handler_t * pHandle, uint8_t * tx_buffer, uint8_t size);

/* Function for initializing the uart0 instance */
void LCD_Receive(LCD_handler_t * pHandle, uint8_t * rx_buffer, uint8_t size);

#endif
