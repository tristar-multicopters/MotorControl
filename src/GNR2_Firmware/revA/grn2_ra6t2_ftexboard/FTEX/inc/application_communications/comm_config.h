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
#include "uCAL_UART.h"
#include "lcd_apt_comm.h"
#include "log_high_speed.h"
#include "co_can_ra6t2.h"
#include "co_timer_ra6t2.h"
#include "board_hardware.h"

//============================ DEFINES ============================//


//========================= EXTERN TYPES ==========================//

extern CO_NODE CONodeGNR;
extern SlaveMotorHandle_t SlaveM2;

extern APT_Handle_t  LCD_APT_handle;
extern UART_Handle_t  UART0Handle;
extern LogHighSpeed_Handle_t LogHS_handle;

#endif /* __COMM_CONFIG_H */
