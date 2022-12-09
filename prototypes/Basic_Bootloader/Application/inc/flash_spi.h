/**
  * @file    flash_spi.h
  * @brief   This file contain the external flash spi driver header.
  *
  */
	
#ifndef FLASH_SPI_H_
#define FLASH_SPI_H_

#include "bsp_api.h"


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "hal_data.h"
/* Defines --------------------------------------------------------------------*/
#define SPI_TRANSFER_SIZE			16

/* Prototypes ----------------------------------------------------------------*/

void FLASH_SPI_INIT();
void FLASH_SPI_CS_HIGH();
void FLASH_SPI_CS_LOW();
uint8_t FLASH_SPI_IO_WriteByte( uint8_t );


FSP_HEADER
FSP_FOOTER

#endif /* FLASH_SPI_H_ */