/**
  ******************************************************************************
  * @file    external_flash.h
  * @brief   This file contain the external flash bsp driver header
  ******************************************************************************
*/
#ifndef __EXTERNAL_FLASH_H
#define __EXTERNAL_FLASH_H

// =============================== Includes ================================ //
#include "mx25_driver.h"
#include "stdint.h"
#include "bsp_api.h"

// ================================ Defines =============================== //
/* FLASH ADDRESSE */
#define HEADER_ADDRESS		0x0000

/* Return for flash functions */
#define FLASH_OK 0
#define FLASH_ERROR 1

// ==================== Public function prototypes ======================== //

/**
  * @brief  Initializes peripherals used by the External Flash device.
  * @param  None
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t ExternalFash_Init(void);

#ifndef LFS_READONLY
/**
  * @brief  Writes block of data to the External Flash. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  pData: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  uwStartAddress: FLASH's internal address to write to.
  * @param  uwDataSize: number of bytes to write to the FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t BSP_SERIAL_FLASH_WriteData(uint32_t WriteAddr, uint8_t* pData, uint32_t Size);
#endif

/**
  * @brief  Reads a block of data from the External Flash.
  * @param  pData: pointer to the buffer that receives the data read from the External Flash.
  * @param  uwStartAddress: FLASH's internal address to read from.
  * @param  uwDataSize: number of bytes to read from the FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t ExternalFash_ReadData(uint32_t uwStartAddress, uint8_t* pData, uint32_t uwDataSize);

/**
  * @brief  Reads External Flash identification.
  * @param  pData: pointer to the buffer that receives the data read from the External Flash.
  * @retval FLASH identification
  */
uint8_t ExternalFash_ReadID(uint8_t* pData);

#endif /* __EXTERNAL_FLASH_H */