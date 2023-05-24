/**
  ******************************************************************************
  * @file    serial_response.c
  * @brief   This file contain the Serial Communication bsp driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "serial_response.h"

// ==================== Public function prototypes ======================== //

/**
 *  Send Acknowledge after data verification 
 */
void Serial_Acknowledge(uint8_t Code_Byte)
{
	uint8_t buffer[4];
	/* Start Byte */
	buffer[0] = 'S';
	/* Acknowledge Byte */
	buffer[1] = 'A';
	/* Code Byte */
	buffer[2] = Code_Byte;
	/* Confirmation Byte */
	buffer[3] = '!';
	/* Send acknowledge message */
	UART_SerialPrint((uint8_t *)buffer);
    UART_SerialPrint((uint8_t *)"\r\n");
}

/**
 *  Send Error after data verification 
 */
void Serial_Error(uint8_t Error_Byte)
{
	uint8_t buffer[4];
	/* Start Byte */
	buffer[0] = 'S';
	/* Error Byte */
	buffer[1] = 'E';
	/* Code Byte */
	buffer[2] = Error_Byte;
	/* Confirmation Byte */
	buffer[3] = '!';
	/* Send error message */
	UART_SerialPrint((uint8_t *)buffer);
    UART_SerialPrint((uint8_t *)"\r\n");
}

/**
 *  Send Response after data verification 
 */
void SEND_RESP(uint8_t Code_Byte,uint8_t Temp_MSG[] ,uint8_t MSG_Length)
{
	uint8_t buffer[MSG_Length+6];
	/* Start Byte */
	buffer[0] = 'S';
	/* Response Byte */
	buffer[1] = 'R';
	/* Error Code Byte */
	buffer[2] = Code_Byte;
	/* Message length Byte */
	buffer[3] = MSG_Length;
	for ( int i=0; i < MSG_Length; i++ )
		{buffer[i+4] = Temp_MSG[i];}
	/* Confirmation Byte */
	buffer[4] = '!';
	/* Send Response message */
	UART_SerialPrint((uint8_t *)buffer);
    UART_SerialPrint((uint8_t *)"\r\n");
}

