/**
  ******************************************************************************
  * @file    serial_flash.h
  * @author  FTEX inc
  * @brief   Header for the serial flash 
  *          This module is the driver for the external SPI memory
  *          using the MX25 driver to manage the flash
  ******************************************************************************
*/

#ifndef __SERIAL_FLASH_H
#define __SERIAL_FLASH_H

// =============================== Includes ================================= //
#include <stdint.h>
#include <cmsis_os2.h>
#include "mx25l3233f_driver.h"

// =============================== Defines ================================= //
#define FLASH_OK        0x00
#define FLASH_ERROR     0x01

typedef struct
{
    MX25_Handle_t   eFlash;
} EFlash_Handle_t;
// ================= Structure used to configure a pin ===================== //


// ==================== Public function prototypes ========================= //
/**
* @brief  Initializes peripherals used by the Serial FLASH device.
* @param  EFlash_Handle_t: serial flash driver handler 
* @retval FLASH_OK (0x00) if operation is correctly performed, else 
*         return FLASH_ERROR (0x01).
*/
uint8_t Serial_Flash_Init(EFlash_Handle_t * pHandle);

/**
  * @brief  Erases the specified FLASH sector.
  * @param  EFlash_Handle_t: serial flash driver handler 
  * @param  SectorAddr: address of the sector to erase.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t Serial_Flash_EraseSector(EFlash_Handle_t * pHandle, uint32_t SectorAddr);

/**
  * @brief  Erases the entire FLASH.
  * @param  EFlash_Handle_t: serial flash driver handler 
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t Serial_Flash_EraseChip(EFlash_Handle_t * pHandle);

/**
  * @brief  Writes block of data to the FLASH. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  EFlash_Handle_t: serial flash driver handler 
  * @param  pData: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  uwStartAddress: FLASH's internal address to write to.
  * @param  uwDataSize: number of bytes to write to the FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t Serial_Flash_WriteData(EFlash_Handle_t * pHandle, uint32_t WriteAddr, uint8_t* pData, uint32_t Size);

/**
  * @brief  Reads a block of data from the FLASH. 
  * @param  EFlash_Handle_t: serial flash driver handler 
  * @param  pData: pointer to the buffer that receives the data read from the FLASH.
  * @param  uwStartAddress: FLASH's internal address to read from.
  * @param  uwDataSize: number of bytes to read from the FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
uint8_t Serial_Flash_ReadData(EFlash_Handle_t * pHandle, uint32_t uwStartAddress, uint8_t* pData, uint32_t uwDataSize);

/**
  * @brief  Reads FLASH identification.
  * @param  EFlash_Handle_t: serial flash driver handler 
  * @param  pData: pointer to the buffer that receives the data read from the FLASH.
  * @retval FLASH identification
  */
uint8_t Serial_Flash_ReadID (EFlash_Handle_t * pHandle, uint8_t* pData);

#endif /*__SERIAL_FLASH_H*/

