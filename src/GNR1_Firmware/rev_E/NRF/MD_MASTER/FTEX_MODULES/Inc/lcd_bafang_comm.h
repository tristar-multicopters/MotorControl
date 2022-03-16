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
#include "board_hardware.h"

#define LCD_BAFFANG_FLAG   				0x10
#define LCD_PENDING_BUFFER_SIZE		4


/*STRUCTURES*/

typedef struct
{
	UFCP_Handle_t lcd_ufcp_handle; // ufcp handle for lcd display
	VCI_Handle_t * pVCInterface;	 // pointer to the vehicle interface
	uint16_t hStatus;							 
	uint16_t hError;
} LCD_handle_t;

/* FUNCTIONS */

void LCD_Baf_Init(LCD_handle_t * pHandle);
__NO_RETURN void TSK_LCD_Baf_comm (void * pvParameter);

#endif
