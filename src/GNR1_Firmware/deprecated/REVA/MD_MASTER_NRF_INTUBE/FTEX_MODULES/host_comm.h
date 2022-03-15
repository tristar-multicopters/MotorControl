/**
  ******************************************************************************
  * @file    host_comm.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles UART frame-based communication with host system (PC, smartphone, ...).
  *
	******************************************************************************
	*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HOST_COMM_H
#define __HOST_COMM_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "md_code_reg.h"
#include "uart_frame_communication_protocol.h"
#include "nrfx_timer.h"
#include "app_uart.h"

#include "md_comm.h"

	
void host_comm_init(volatile md_reg_t * host_reg);

void TSK_HOSTreceiveFrames (void * pvParameter);


#endif /*__HOST_COMM_H*/
