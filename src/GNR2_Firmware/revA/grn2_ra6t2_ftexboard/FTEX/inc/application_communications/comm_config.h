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
#include "uCAL_DATAFLASH.h"

#include "uCAL_SPI.h"
#include "MX25L3233F_driver.h"
#include "serial_flash.h"
#include "serial_flash_storage.h"

//============================ DEFINES ============================//


//========================= EXTERN TYPES ==========================//

extern CO_NODE CONodeGNR;
extern SlaveMotorHandle_t SlaveM2;

extern APT_Handle_t  LCD_APT_handle;
extern UART_Handle_t  UART0Handle;
extern LogHighSpeed_Handle_t LogHS_handle;
extern DataFlash_Handle_t DataFlashHandle;

extern SPI_Handle_t	SPI1Handle;
extern EFlash_Storage_Handle_t EFlash_Storage_Handle;

#endif /* __COMM_CONFIG_H */
