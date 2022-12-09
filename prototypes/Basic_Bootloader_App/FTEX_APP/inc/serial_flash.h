/**
  * @file    serial_flash.h
  * @brief   This file contain the header for the serial 
	*					 flash driver for communication
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERIAL_FLASH_H
#define __SERIAL_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "mx25l3233f.h"
#include "stdint.h"
#include "bsp_api.h"
/* Defines --------------------------------------------------------------------*/

// Return for flash functions
#define FLASH_OK 0
#define FLASH_ERROR 1

/* Function Prototypes ------------------------------------------------------------*/

uint8_t BSP_SERIAL_FLASH_Init(void);
uint8_t BSP_SERIAL_FLASH_EraseSector(uint32_t SectorAddr);
uint8_t BSP_SERIAL_FLASH_EraseChip(void);
uint8_t BSP_SERIAL_FLASH_WriteData(uint32_t uwStartAddress, uint8_t* pData, uint32_t uwDataSize);
uint8_t BSP_SERIAL_FLASH_ReadData( uint32_t uwStartAddress, uint8_t* pData, uint32_t uwDataSize);
uint8_t BSP_SERIAL_FLASH_ReadID(uint8_t* pData);




#endif /* __SERIAL_FLASH_H */
