/**
  ******************************************************************************
  * @file    evionics.h
	* @author  Jorge Andres Polo, FTEX
  * @brief   Header of evionics.c module that controls communication between host  				 
  *					 (Evionics interface) and the GNR	
	******************************************************************************
	*/

#include "board_hardware.h"
#include "storage_management.h" // it includes vc_interface which includes euart_manager.h
#include "vc_interface.h"
#include "euart_manager.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EVIONICS_H
#define __EVIONICS_H

#define DATA_SIZE	0x100 										 // 256 bytes (64 int32_t parameters)
#define	CRC_SIZE	2													 // CRC size
#define EV_RXBUFF_SIZE  DATA_SIZE + CRC_SIZE // Total of max bytes expected to receive in the buffer
/* State machine stages for receiving data from EVionics*/
typedef enum
{
	EV_FRAME_ID,  /* Recieving Frame ID */
	EV_SIZE,			/* Recieving Size ID  */
	EV_DATA_CRC	  /* Receiving Data ID and CRC */
} EV_rx_stage_t;

/* Command ID byte type*/
typedef enum
{
	EV_SET_REG, 	 	  /* Command for setting a register 								 */
	EV_GET_REG, 		  /* Command for getting the value of a register     */
	EV_LOAD_PARAMS,  	/* Command for updating the flash memory registers */ 
	EV_FLASH_MEMORY   /* Command for flashing the memory                 */
}EV_command_t;

/* Frame ID for register sections */
typedef enum
{
	VEHICLE_SECT_1 = 1, 		 /* Reg address from 1 to 64 	     */
	VEHICLE_SECT_2 = 65,		 /* Reg address from 65 to 128  	 */
	VEHICLE_SECT_3 = 129,		 /* Reg address from 129 to 192 	 */
	VEHICLE_SECT_4 = 193,		 /* Reg address from 193 to 256 	 */
	
	PERIPHERAL_SECT_1 = 257, /* Reg address from 297 to 320 	 */
	PERIPHERAL_SECT_2 = 321, /* Reg address from 321 to 384 	 */
	PERIPHERAL_SECT_3 = 385, /* Reg address from 385 to 448 	 */
	PERIPHERAL_SECT_4 = 449, /* Reg address from 449 to 512 	 */
	
	BATTERY_SECT_1 = 513, 	 /* Reg address from 513 to 576 	 */
	BATTERY_SECT_2 = 577, 	 /* Reg address from 577 to 640 	 */
	BATTERY_SECT_3 = 641, 	 /* Reg address from 641 to 704 	 */
	BATTERY_SECT_4 = 705, 	 /* Reg address from 705 to 768 	 */
	
	MOTOR_SECT_1 = 769, 		 /* Reg address from 769 to 832 	 */
	MOTOR_SECT_2 = 833, 		 /* Reg address from 833 to 896 	 */
	MOTOR_SECT_3 = 897, 		 /* Reg address from 897 to 960 	 */
	MOTOR_SECT_4 = 961, 		 /* Reg address from 961 to 1024  */
	
	COMM_SECT_1 = 1025, 		 /* Reg address from 1025 to 1088  */
	COMM_SECT_2 = 1089, 		 /* Reg address from 1089 to 1152  */
	COMM_SECT_3 = 1153, 		 /* Reg address from 1153 to 1216  */
	COMM_SECT_4 = 1217, 		 /* Reg address from 1217 to 1280  */
	
	MANUF_SECT_1 = 1281, 		 /* Reg address from 1281 to 1344  */
	MANUF_SECT_2 = 1345, 		 /* Reg address from 1345 to 1408  */
	MANUF_SECT_3 = 1409, 		 /* Reg address from 1409 to 1472  */
	MANUF_SECT_4 = 1473, 		 /* Reg address from 1473 to 1536  */
} EV_regSectors_t;

typedef struct
{
	uint8_t cmd; 												 // Gives the received command type
	uint16_t code;                       // Describes the type of action
  uint8_t size;                        // Size of the Payload of the frame in bytes.
  uint8_t buffer[EV_RXBUFF_SIZE];    	 // buffer containing the received data.
	uint16_t ByteCnt;									   // Counter of bytes
	EV_rx_stage_t currentStage;
}EVionics_frame_rx;

typedef struct
{
	uint8_t tx_code; 								 		 // Code for knowing if the frame has been well received or not
  uint8_t size;                        // Size of the Payload of the frame in bytes.
  uint8_t buffer[8];    	 					   // buffer containing the data to answer.
	uint16_t ByteCnt;									   // Counter of bytes to send
}EVionics_frame_tx;

typedef struct
{
	EVionics_frame_rx rx_frame; 			// Frame for data reception
	EVionics_frame_tx tx_frame;				// Frame for data transmission
	eUART_handler_t euart_handler; 		// Contains the callback that will be assigned
																		// to the event_handler of EVionics
	VCI_Handle_t *p_VController;      // Pointer to the vehicle
}EVionics_handle_t;

/************************* FUNCTIONS **************************/
/* Function for initialise the eUart module with EVionics configuration */
void EVionics_init(VCI_Handle_t* pHandle);

/* Function for decoding the received data frame */
void EVionics_frame_process( void );
#endif
