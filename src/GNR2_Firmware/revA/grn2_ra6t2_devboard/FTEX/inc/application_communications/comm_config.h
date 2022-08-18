/**
  * @file    comm_config.h
  * @author  FTEX
  * @brief   This module declares global structures used by 
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMM_CONFIG_H
#define __COMM_CONFIG_H

#include "vc_interface.h"
#include "comm_parameters.h"
#include "uCAL_UART.h"
#include "lcd_apt_comm.h"
#include "co_can_ra6t2.h"
#include "co_timer_ra6t2.h"

//============================ DEFINES ============================//
// FLAGS
#define CAN_RX_FLAG 0x20 

//========================= EXTERN TYPES ==========================//
extern APT_Handle_t  LCD_APT_handle;
extern UART_Handle_t  UART0_handle;
extern CAN_Handler_t CAN_handle;
// Thread for CANProcessMsgs task
extern osThreadId_t CANRxFrameHandle;
#endif /* __COMM_CONFIG_H */

