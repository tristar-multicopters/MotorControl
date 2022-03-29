/**
  ******************************************************************************
  * @file    ev_config_tool.h
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
/* State machine stages for receiving data from EVCT*/
typedef enum
{
	EV_FRAME_ID,  /* Receiving Frame ID */
	EV_SIZE,			/* Receiving Size ID  */
	EV_DATA_CRC	/* Receiving Data ID and CRC */
} EV_rx_stage_t;

/* Command ID byte type*/
typedef enum
{
	EV_SET_REG, 	 	  /* Command for setting a register 								 */
	EV_GET_REG, 		  /* Command for getting the value of a register     */
	EV_LOAD_PARAMS,  	/* Command for updating the flash memory registers */ 
}EV_command_t;

/* Frame ID for register sections */
typedef enum
{
	VEHICLE_SECT_1 = 0, 		 /* Reg address from 0 to 63 	     */
	VEHICLE_SECT_2 = 64,		 /* Reg address from 64 to 127  	 */
	VEHICLE_SECT_3 = 128,	   /* Reg address from 128 to 193 	 */
	VEHICLE_SECT_4 = 192,	   /* Reg address from 194 to 255 	 */

	PERIPHERAL_SECT_1 = 256, /* Reg address from 256 to 319 	 */
	PERIPHERAL_SECT_2 = 320, /* Reg address from 320 to 383 	 */
	PERIPHERAL_SECT_3 = 384, /* Reg address from 384 to 447 	 */
	PERIPHERAL_SECT_4 = 448, /* Reg address from 4478 to 511	 */

	BATTERY_SECT_1 = 512,    /* Reg address from 512 to 575 	 */
	BATTERY_SECT_2 = 576,    /* Reg address from 576 to 639 	 */
	BATTERY_SECT_3 = 640,    /* Reg address from 640 to 703 	 */
	BATTERY_SECT_4 = 704,    /* Reg address from 704 to 767 	 */

	MOTOR_SECT_1 = 768, 		 /* Reg address from 768 to 831 	 */
	MOTOR_SECT_2 = 832, 		 /* Reg address from 832 to 897 	 */
	MOTOR_SECT_3 = 896, 		 /* Reg address from 898 to 959 	 */
	MOTOR_SECT_4 = 960, 		 /* Reg address from 960 to 1023   */

	COMM_SECT_1 = 1024, 		 /* Reg address from 1024 to 1087  */
	COMM_SECT_2 = 1088, 		 /* Reg address from 1088 to 1151  */
	COMM_SECT_3 = 1152, 		 /* Reg address from 1152 to 1215  */
	COMM_SECT_4 = 1216, 		 /* Reg address from 1216 to 1279  */

	MANUF_SECT_1 = 1280, 	 	 /* Reg address from 1280 to 1343  */
	MANUF_SECT_2 = 1344, 	   /* Reg address from 1344 to 1407  */
	MANUF_SECT_3 = 1408, 	   /* Reg address from 1408 to 1471  */
	MANUF_SECT_4 = 1472, 	   /* Reg address from 1472 to 1535  */
} EV_regSectors_t;

typedef struct
{
	uint8_t cmd; 												 // Gives the received command type
	uint16_t code;                       // Describes the type of action
  uint8_t size;                        // Size of the Payload of the frame in bytes.
  uint8_t buffer[EV_RXBUFF_SIZE];    	 // buffer containing the received data.
	uint16_t ByteCnt;									   // Counter of bytes
	EV_rx_stage_t currentStage;
}EVCT_frame_rx;

typedef struct
{
	uint8_t tx_code; 								 		 // Code for knowing if the frame has been well received or not
  uint8_t size;                        // Size of the Payload of the frame in bytes.
  uint8_t buffer[8];    	 					   // buffer containing the data to answer.
	uint16_t ByteCnt;									   // Counter of bytes to send
}EVCT_frame_tx;

typedef struct
{
	EVCT_frame_rx rx_frame; 			// Frame for data reception
	EVCT_frame_tx tx_frame;				// Frame for data transmission
	eUART_handler_t euart_handler; 		// Contains the callback that will be assigned
																		// to the event_handler of EVCT
	VCI_Handle_t *p_VController;      // Pointer to the vehicle
}EVCT_handle_t;

/************************* FUNCTIONS **************************/
/* Function for initialise the eUart module with EVCT configuration */
void EVCT_init(VCI_Handle_t* pHandle);

/* Function for decoding the received data frame */
void EVCT_frame_process( void );
#endif
