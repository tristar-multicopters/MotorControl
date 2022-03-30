/**
  ******************************************************************************
  * @file    lcd_apt_comm.h
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes APT LCD communication protocol.
  *
	******************************************************************************
	*/
	
#ifndef __LCD_APT_COMM_H
#define __LCD_APT_COMM_H

#include "vc_interface.h"

/**************************** DEFINITIONS AND STRUCTS ****************************/
// Commands
#define APT_START      0x55
#define APT_END        0x0D

//Display Read Cmd
 typedef enum{
 PASS     = 1,
 SPEED    = 2,
 CURRENTL = 3,
 WHEELD   = 4, 
 SENSOR   = 5,
 CHECK    = 6 
}APT_Receive;
 
/*//Display Write Cmd
#define W_SPEED_LIMIT 0x1F
#define W_ASSIST      0x0B 

//Assist levels
#define A_0      0x00 //No assist
#define A_1      0x01
#define A_2      0x0B //First(5)  
#define A_3      0x0C
#define A_4      0x0D //Seconde(5)
#define A_5      0x02  
#define A_6      0x15 //Third(5)
#define A_7      0x16
#define A_8      0x17 //Fourth(5) 
#define A_9      0x03 //Max assist
#define A_PUSH   0x06 //Push bike 
#define A_LSPEED 0x0A //Limit speed*/

#define APT_MAX_BUFF_SIZE 13  // Max size for frame buffer.

typedef struct
{                         
  uint8_t Size;                         // Size of the Payload of the frame in bytes.
  uint8_t Buffer[APT_MAX_BUFF_SIZE];    // buffer containing the received data.
	uint8_t ByteCnt;
}APT_frame_t;

typedef struct
{
	APT_frame_t rx_frame; 		   	 // Frame for data reception
	APT_frame_t tx_frame; 		   	 // Frame for send response
	eUART_handler_t euart_handler; // Contains the callback that will be assigned 
																 // to the event_handler and the type of LCD
	VCI_Handle_t *pVController;   	 // Pointer to vehicle
}APT_Handle_t;

/********************************* FUNCTIONS *******************************/
/* Initialisation of Bafang protocol */
void LCD_APT_init(VCI_Handle_t * pHandle);
/*Function for building the frame */
void * LCD_APT_RX_IRQ_Handler(unsigned short rx_data);
/*Function for sending a response */
void LCD_APT_TX_IRQ_Handler(void);
/* Function for decoding the received frame */
void LCD_APT_frame_Process( void );

#endif
