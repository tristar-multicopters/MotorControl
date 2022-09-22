/**
  ******************************************************************************
  * @file    serial_flash_stoarage.h
  * @author  FTEX inc
  * @brief   Header to serial flash storage 
  *          This module is the interface that is used to interact with the 
  *          external SPI Memory for pack/file storage
  ******************************************************************************
*/

#ifndef __SERIAL_FLASH_STORAGE_H
#define __SERIAL_FLASH_STORAGE_H

// =============================== Includes ================================= //
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "serial_flash.h"

// =============================== Defines ================================= //
#define HEADER_SIZE             40
#define HEADER_START_ADDR       0x00000000
#define FW_START_ADDR           (HEADER_START_ADDR + HEADER_SIZE)
#define FW_ADDR(n)              (FW_START_ADDR + n)

// ================= Structure used to configure a pin ===================== //

typedef struct
{
    EFlash_Handle_t  eFlashStorage;
} EFlash_Storage_Handle_t;

// ==================== Public function prototypes ========================= //

/**
* @brief  Initializes the serial flash storage
* @param  EFlash_Storage_Handle_t : serial flash storage handler
* @retval Serial Flash Storage Status
*/
uint8_t SF_Storage_Init(EFlash_Storage_Handle_t * pHandle);


#endif // __SERIAL_FLASH_STORAGE_H