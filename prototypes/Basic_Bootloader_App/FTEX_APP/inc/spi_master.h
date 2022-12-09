/**
  * @file    spi_master.h
  * @brief   This file contaisn the header for the external flash spi driver.
  */

#ifndef __SPI_MASTER_H
#define __SPI_MASTER_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Macros --------------------------------------------------------------------*/
#define SPI_TRANSFER_SIZE			16

/* Prototypes ----------------------------------------------------------------*/

void FLASH_SPI_INIT(void); 
void FLASH_SPI_CS_HIGH();
void FLASH_SPI_CS_LOW();
uint8_t FLASH_SPI_IO_WriteByte( uint8_t );


#endif /* __SPI_MASTER_H */
