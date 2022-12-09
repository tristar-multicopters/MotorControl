/**
  * @file    flash_spi.c
  * @brief   This file contaisn the external flash spi driver.
  *
  */
	
/* Includes ------------------------------------------------------------------*/
#include "flash_spi.h"

/* Variables -----------------------------------------------------------------*/

uint8_t tx_buffer[SPI_TRANSFER_SIZE];
uint8_t rx_buffer[SPI_TRANSFER_SIZE];
extern volatile bool g_transfer_complete;


/* Public Functions ----------------------------------------------------------*/

/**
	Function used to desactivate the CS SPI Protocol
*/
void FLASH_SPI_INIT()
{
	R_SPI_B_Open(&g_spi1_ctrl, &g_spi1_cfg);
}

/**
	Function used to desactivate the CS SPI Protocol
*/
void FLASH_SPI_CS_HIGH()
{
	R_IOPORT_PinWrite(&g_ioport_ctrl, SPI_CS, BSP_IO_LEVEL_HIGH);
}

/**
	Function used to activate the CS SPI Protocol
*/
void FLASH_SPI_CS_LOW()
{
	R_IOPORT_PinWrite(&g_ioport_ctrl, SPI_CS, BSP_IO_LEVEL_LOW);
}

/**
	Function used to read/write byte via SPI Protocol
*/
uint8_t FLASH_SPI_IO_WriteByte( uint8_t byte )
{
	uint8_t timeout=0;
	fsp_err_t err = FSP_SUCCESS;
	g_transfer_complete = false;
	
	err = R_SPI_B_WriteRead(&g_spi1_ctrl, &byte, rx_buffer, 1, SPI_BIT_WIDTH_8_BITS);
	
	assert(FSP_SUCCESS == err);
	
	/* Wait for SPI_EVENT_TRANSFER_COMPLETE callback event. */
	while (false == g_transfer_complete)
	{
		 R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);
		 if(timeout++ >= 100)
		 {
			 g_transfer_complete = true;
		 }
	}
	return rx_buffer[0];
}
