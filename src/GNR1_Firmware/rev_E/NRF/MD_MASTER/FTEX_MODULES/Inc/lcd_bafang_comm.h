/**
  ******************************************************************************
  * @file    lcd_bafang_comm.h
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   High level module that describes Bafang LCD communication protocol.
  *
	******************************************************************************
	*/
	
#ifndef __LCD_BAFANG_COMM_H
#define __LCD_BAFANG_COMM_H

#include "vc_interface.h"

/**************************** DEFINITIONS AND STRUCTS ****************************/
// Commands
#define BAF_CMD_READ      0x11
#define BAF_CMD_WRITE     0x16

//Display Read Cmd
#define SKIP          0x00
#define R_VERSION     0x90
#define R_STATUS      0x08
#define R_WORKSTATUS  0x31 
#define R_CURRENT     0x0A
#define R_BATCAP      0x11
#define R_RSPEED      0x20
#define R_LIGHTSTATUS 0x1B
#define R_PHOTTHRESH  0x1C
//Display Write Cmd
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
#define A_LSPEED 0x0A //Limit speed

#define BAF_MAX_BUFF_SIZE 8  // Max size for reception buffer.

typedef struct
{
	uint8_t Code;                         // Describes the received command (BAF_CMD_READ or BAF_CMD_WRITE)
  uint8_t Size;                         // Size of the Payload of the frame in bytes.
  uint8_t Buffer[BAF_MAX_BUFF_SIZE];    // buffer containing the received data.
	uint8_t ByteCnt;
}BAF_frame_t;

typedef struct
{
	BAF_frame_t rx_frame; 		   	 // Frame for data reception
	BAF_frame_t tx_frame; 		   	 // Frame for send response
	eUART_handler_t euart_handler; // Contains the callback that will be assigned 
																 // to the event_handler and the type of LCD
	VCI_Handle_t *pVController;    // Pointer to vehicle
  uint8_t TrashCnt;              // Used to skip wave of trash on screen boot up
}BAF_Handle_t;

/********************************* FUNCTIONS *******************************/
/* Initialisation of Bafang protocol */
void LCD_BAF_init(VCI_Handle_t * pHandle);
/*Function for building the frame */
void * LCD_BAF_RX_IRQ_Handler(unsigned short rx_data);
/*Function for sending a response */
void LCD_BAF_TX_IRQ_Handler(void);
/* Function for decoding the received frame */
void LCD_BAF_frame_Process( void );

#endif
