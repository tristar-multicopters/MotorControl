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

#define EV_MAX_SIZE	0x100 // 256 int32_t parameters (1024 bytes)

/* State machine stages for receiving data from EVionics*/
typedef enum
{
	EV_FRAME_ID,  /* Recieving Frame ID */
	EV_SIZE,			/* Recieving Size ID  */
	EV_DATA,			/* Recieving Data ID  */
	EV_CRC,				/* Recieving CRC Data */
} EV_rx_stage_t;

/* Command ID byte type*/
typedef enum
{
	EV_SET_REG, 	/* Command for setting a register 								 */
	EV_GET_REG, 	/* Command for getting the value of a register     */
	EV_FLASH_GNR  /* Command for updating the flash memory registers */ 
}EV_command_t;

typedef struct
{
	uint8_t cmd; 												 // Gives the received command type
	uint16_t code;                       // Describes the type of actio
  uint8_t size;                        // Size of the Payload of the frame in bytes.
  int32_t buffer[EV_MAX_SIZE];    	   // buffer containing the received data.
	uint16_t ByteCnt;									   // Counter of bytes
	EV_rx_stage_t currentStage;
}EVionics_frame_rx;

typedef struct
{
	EVionics_frame_rx rx_frame; 			// Frame for data reception
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
