/**
  ******************************************************************************
  * @file    lcd_eggrider_comm.h
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   High level module that describes Eggrider LCD communication protocol.
  *
	******************************************************************************
	*/
	
#ifndef __LCD_EGGRIDER_COMM_H
#define __LCD_EGGRIDER_COMM_H

#include "vc_interface.h"

/**************************** DEFINITIONS AND STRUCTS ****************************/
// Commands
#define EGG_CMD_READ      0x50
#define EGG_CMD_WRITE     0x55
#define EGG_CMD_CUSTOM    0x5E

#define EGG_STANDARD_READ 0x01
#define EGG_STANDARD_READ_SIZE  0x04

//Assist levels

#define EGG_MAX_BUFF_SIZE 64  // Max size for reception and transmission buffer.

typedef struct
{
	uint8_t Code;                         // Describes the received command (EGG_CMD_READ or EGG_CMD_WRITE)
  uint8_t Size;                         // Size of the Payload of the frame in bytes.
  uint8_t Buffer[EGG_MAX_BUFF_SIZE];    // buffer containing the received data.
	uint8_t ByteCnt;
}EGG_frame_t;



typedef struct
{
	EGG_frame_t rx_frame; 		   	 // Frame for data reception
	EGG_frame_t tx_frame; 		   	 // Frame for send response
	eUART_handler_t euart_handler; 	 // Contains the callback that will be assigned 
																 // to the event_handler and the type of LCD
	VC_Handle_t *pVController;   	 // Pointer to vehicle
}EGG_Handle_t;
/***************************************************************************/

/********************************* FUNCTIONS *******************************/
/* Initialisation of EggRider protocol */
void LCD_EGG_init(VC_Handle_t * pHandle);

void LCD_EGG_TX_IRQ_Handler(void);

/*Function for building the frame */
void * LCD_EGG_RX_IRQ_Handler(unsigned short rx_data);

/* Function for decoding the received frame */
void LCD_EGG_frame_Process( void );

uint16_t LCD_EGG_CRC16(EGG_frame_t frame);

#endif
