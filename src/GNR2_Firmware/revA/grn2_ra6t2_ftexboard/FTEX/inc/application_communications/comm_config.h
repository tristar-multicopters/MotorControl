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
    #if SCREEN_PROTOCOL == UART_APT
        #include "lcd_apt.h"
        extern APT_Handle_t  LCD_APT_handle;
    #elif SCREEN_PROTOCOL == UART_KD718
        #include "lcd_KD718.h"
        extern KD718_Handle_t LCD_KD718_handle;
    #elif SCREEN_PROTOCOL == UART_CLOUD_5S
        #include "lcd_Cloud_5S.h"
        extern Cloud_5S_Handle_t LCD_Cloud_5S_handle;
    #elif SCREEN_PROTOCOL == UART_LOG_HS
        #include "log_high_speed.h"
        extern LogHighSpeed_Handle_t LogHS_handle;
    #else
    #endif 


extern CO_NODE CONodeGNR;
extern SlaveMotorHandle_t SlaveM2;
extern UART_Handle_t  UART0Handle;







extern DataFlash_Handle_t DataFlashHandle;

extern SPI_Handle_t	SPI1Handle;
extern EFlash_Storage_Handle_t EFlash_Storage_Handle;

#endif /* __COMM_CONFIG_H */
