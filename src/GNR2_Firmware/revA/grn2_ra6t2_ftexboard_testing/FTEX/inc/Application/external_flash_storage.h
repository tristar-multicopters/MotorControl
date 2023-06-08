/**
  ******************************************************************************
  * @file    external_flash_storage.h
  * @author  FTEX inc
  * @brief   Header to external flash storage 
  *          This module is the interface that is used to interact with the 
  *          external SPI Memory
  ******************************************************************************
*/

#ifndef __INTERNAL_FLASH_STORAGE_H
#define __INTERNAL_FLASH_STORAGE_H

// =============================== Includes ================================= //


// ==================== Public function prototypes ========================= //

/**
* @brief  Initializes the serial flash storage
* @param  EFlash_Storage_Handle_t : serial flash storage handler
* @retval Serial Flash Storage Status
*/
uint8_t SF_format(void);

#endif // __EXTERNAL_FLASH_STORAGE_H