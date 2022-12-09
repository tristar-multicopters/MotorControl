/**
  * @file    serial_flash.h
  * @brief   This file is the serial flash driver
	*/

#ifndef __SERIAL_FLASH_H
#define __SERIAL_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "mx25_driver.h"
#include "stdint.h"
#include "bsp_api.h"


/* Defines --------------------------------------------------------------------*/

// FLASH ADDRESSE
#define HEADER_ADDRESS		0x0000

// Return for flash functions
#define FLASH_OK 0
#define FLASH_ERROR 1

/* Function Prototypes ------------------------------------------------------------*/


/** ERASE secondary image
     * @summary open the secondary image space and erase it. Used before an update from the flash
     * @param   void
     * @return  src_address    Address of the buffer containing the data to write to Flash.
	**/
int INTERNAL_FLASH_erase_secondary_image(void);

/**
  * @brief  Initializes peripherals used by the Serial FLASH device.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */

uint8_t BSP_SERIAL_FLASH_Init(void);

/**
  * @brief  Writes block of data to the FLASH. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  pData: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  uwStartAddress: FLASH's internal address to write to.
  * @param  uwDataSize: number of bytes to write to the FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
#ifndef LFS_READONLY
uint8_t BSP_SERIAL_FLASH_WriteData(uint32_t WriteAddr, uint8_t* pData, uint32_t Size);
#endif
/**
  * @brief  Reads a block of data from the FLASH.
  * @param  pData: pointer to the buffer that receives the data read from the FLASH.
  * @param  uwStartAddress: FLASH's internal address to read from.
  * @param  uwDataSize: number of bytes to read from the FLASH.
  * @retval FLASH_OK (0x00) if operation is correctly performed, else 
  *         return FLASH_ERROR (0x01).
  */
	
uint8_t BSP_SERIAL_FLASH_ReadData(uint32_t uwStartAddress, uint8_t* pData, uint32_t uwDataSize);

/**
  * @brief  Reads FLASH identification.
  * @retval FLASH identification
  */
	
uint8_t BSP_SERIAL_FLASH_ReadID(uint8_t* pData);




#endif /* __SERIAL_FLASH_H */