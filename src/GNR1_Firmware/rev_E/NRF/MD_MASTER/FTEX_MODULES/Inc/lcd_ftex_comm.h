/**
  ******************************************************************************
  * @file    lcd_ftex_comm.h
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   High level module that describes FTEX LCD communication protocol.
  *
	******************************************************************************
	*/
	
#ifndef __LCD_FTEX_COMM_H
#define __LCD_FTEX_COMM_H

#include "vc_interface.h"

/**************************** DEFINITIONS AND STRUCTS ****************************/
// Commands
#define FTEX_FRAME_READ      0x50
#define FTEX_FRAME_WRITE     0x55
#define FTEX_FRAME_CUSTOM    0x5E

#define FTEX_F_SUBTYPE_01      0x01
#define FTEX_STANDARD_READ_SIZE  0x04

//Assist levels

#define FTEX_MAX_BUFF_SIZE 64  // Max size for reception and transmission buffer.

typedef struct
{
	uint8_t Code;                         // Describes the received command (EGG_CMD_READ or EGG_CMD_WRITE)
  uint8_t Size;                         // Size of the Payload of the frame in bytes.
  uint8_t Buffer[FTEX_MAX_BUFF_SIZE];    // buffer containing the received data.
	uint8_t ByteCnt;
}FTEX_frame_t;



typedef struct
{
	FTEX_frame_t rx_frame; 		   	 // Frame for data reception
	FTEX_frame_t tx_frame; 		   	 // Frame for send response
	eUART_handler_t euart_handler; // Contains the callback that will be assigned 
																 // to the event_handler and the type of LCD
	VCI_Handle_t *pVController;   	 // Pointer to vehicle
}FTEX_Handle_t;
/***************************************************************************/

/********************************* FUNCTIONS *******************************/
/* Initialisation of EggRider protocol */
void LCD_FTEX_init(VCI_Handle_t * pHandle);

void LCD_FTEX_TX_IRQ_Handler(void);

/*Function for building the frame */
void * LCD_FTEX_RX_IRQ_Handler(unsigned short rx_data);

/* Function for decoding the received frame */
void LCD_FTEX_frame_Process( void );

uint16_t LCD_FTEX_CRC16(FTEX_frame_t frame);

#endif
