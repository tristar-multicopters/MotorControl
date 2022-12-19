/**
  ******************************************************************************
  * @file    internal_flash.h
  * @brief   This file contain the Internal Flash driver header.
  ******************************************************************************
*/
#ifndef __INTERNAL_FLASH_H
#define __INTERNAL_FLASH_H

// =============================== Includes ================================ //
#include "bsp_api.h"

// ================================ Defines =============================== //


// ==================== Public function prototypes ======================== //

/**
  * @brief  Erases the internal secondary image.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return BOOT_EFLASH (0x01).
*/
int INTERNAL_FLASH_erase_secondary_image(void);

/**
  * @brief  Write a line of flash data into the internal flash
  * @param	hexline with all the information size/adress and data
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return BOOT_EFLASH (0x01).
*/
int INTERNAL_FLASH_write(uint8_t *pData,	uint32_t address,	uint32_t size);


#endif /* __INTERNAL_FLASH_H */