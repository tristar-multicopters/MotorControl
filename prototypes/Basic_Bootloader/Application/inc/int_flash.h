/**
  * @file    int_flash.h
  * @brief   This file is the internal flash header
	*/

#ifndef __INT_FLASH_H
#define __INT_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "bsp_api.h"

/* Defines --------------------------------------------------------------------*/

#define HEX_INTEL_MAX_DATA	16

typedef struct{
	uint8_t size;
	uint16_t address;
	uint8_t rec;
	uint8_t data[HEX_INTEL_MAX_DATA] ;
	uint8_t crc;
}hex_intel;

/* Function Prototypes ------------------------------------------------------------*/

/*
	* ERASE secondary image
*/
int INTERNAL_FLASH_erase_secondary_image(void);

/*
	* Write data in secondary image
*/
int INTERNAL_FLASH_write(uint8_t *pData,	uint32_t address,	uint32_t size);


#endif /* __BOOT_MAIN_H */