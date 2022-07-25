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
#include "uCAL_CAN.h"
#include "uCAL_TIM1.h"

extern APT_Handle_t  LCD_APT_handle;
extern UART_Handle_t  UART0_handle;
extern CAN_Handler_t CAN_handle;
#endif /* __COMM_CONFIG_H */

