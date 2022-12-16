/**
  ******************************************************************************
  * @file    uCAL_SPI.h
  * @author  FTEX inc
  * @brief   Header for the uCAL_SPI communication protocol
  *          This module is the interface that is used by the entire firmware
  *          to interact with the SPI for communication. 
  ******************************************************************************
*/

#ifndef __UCAL_SPI_H
#define __UCAL_SPI_H

// =============================== Includes ================================= //
#include "stdlib.h"
#include "hal_data.h"

// =============================== Defines ================================= //
#define SPI_TRANSFER_SIZE			16

// ================= Structure used to configure a pin ===================== //
typedef struct
{
    spi_b_instance_ctrl_t * pSPI_Instance;
	
    const spi_bit_width_t pSPI_BitWidth;
	
    uint16_t hSPI_FrameLength;
    uint8_t tx_buffer[SPI_TRANSFER_SIZE];
    uint8_t rx_buffer[SPI_TRANSFER_SIZE];
	
    volatile bool bSPI_transfer_complete;
} SPI_Handle_t;

// ==================== Public function prototypes ========================= //
/**
  * @brief  Function used to read/write via SPI Protocol
  * @param  SPI_Handle_t handle
  * @return rx_buffer in uint8_t
  */
uint8_t uCAL_SPI_IO_WriteByte(SPI_Handle_t * pHandle, uint8_t byte);

/**
  * @brief  Write one byte over the SPI
  * @param  SPI_Handle_t handle
  * @param  byte to send tp the external flash by SPI
  * @retval FSP_SUCCESS (0x00) if operation is correctly performed, else 
  *         return FSP_ERROR (0x01).
  */
uint8_t uCAL_SPI_IO_WriteSingleByte(SPI_Handle_t * pHandle, uint8_t byte);

/**
  * @brief  Read one byte over the SPI
  * @param  SPI_Handle_t handle
  * @retval Byte Read over the SPI
  */
uint8_t uCAL_SPI_IO_ReadSingleByte(SPI_Handle_t * pHandle);

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

#endif	/*__UCAL_SPI_H*/
