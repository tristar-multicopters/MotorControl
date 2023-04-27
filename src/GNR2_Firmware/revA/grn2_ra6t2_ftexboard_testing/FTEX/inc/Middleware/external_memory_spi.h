/**
  ******************************************************************************
  * @file    external_flash_spi.h
  * @brief   This file contain the SPI communication driver header.
  ******************************************************************************
*/

#ifndef __EXTERNAL_MEMORY_SPI_H_
#define __EXTERNAL_MEMORY_SPI_H_

// =============================== Includes ================================ //
#include "stdint.h"
#include "bsp_api.h"
#include "hal_data.h"
// ================================ Defines =============================== //
#define SPI_TRANSFER_SIZE   16
#define SPI_BYTE_SIZE       1

/* SPI Hardware definition */
#define pSPIInstance        &g_spi1_ctrl
#define pSPIInstance_Conf   &g_spi1_cfg
#define pSPIWidth           SPI_BIT_WIDTH_8_BITS

// ==================== Public function prototypes ======================== //

/**
  * @brief  Intialize the SPI communication
  * @param  None
  * @retval FSP_SUCCESS (0x00) if operation is correctly performed, else 
  *         return FSP_ERROR (0x01).
  */
uint8_t Init_spi(void);

/**
  * @brief  Function used to read/write via SPI Protocol
  * @param  None
  * @return rx_buffer in uint8_t
  */
uint8_t uCAL_SPI_WriteByte( uint8_t byte );

/**
  * @brief  Write one byte over the SPI
  * @param  None
  * @param  byte to send tp the external flash by SPI
  * @retval FSP_SUCCESS (0x00) if operation is correctly performed, else 
  *         return FSP_ERROR (0x01).
  */
uint8_t uCAL_SPI_IO_WriteSingleByte(uint8_t byte);

/**
  * @brief  Function used to desactivate the CS SPI Protocol
  * @param  SPI_Handle_t handle
  * @return None
  */
void uCAL_SPI_Disable(void);

/**
  * @brief  Function used to activate the CS SPI Protocol
  * @param  SPI_Handle_t handle
  * @return None
  */
void uCAL_SPI_Enable(void);

#endif /* __EXTERNAL_MEMORY_SPI_H_ */