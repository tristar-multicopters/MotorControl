/**
  ******************************************************************************
  * @file    uCAL_SPI.c
  * @brief   This file contain the SPI communication driver.
  ******************************************************************************
*/

// =============================== Includes ================================ //
#include "uCAL_SPI.h"
#include "ASSERT_FTEX.h"

// ========================== Public Variables ============================ //
uint8_t rx_buffer[SPI_TRANSFER_SIZE];
extern volatile bool bTransfer_Complete;

// ==================== Public function prototypes ======================== //

/**
	Function used to desactivate the CS SPI Protocol
*/
uint8_t uCAL_SPI_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;
	err = R_SPI_B_Open(pSPIInstance, pSPIInstance_Conf);
	return (uint8_t) err;
}

/**
	Function used to read/write byte via SPI Protocol
*/
uint8_t uCAL_SPI_WriteByte(uint8_t byte)
{
	uint8_t timeout=0;
	fsp_err_t err = FSP_SUCCESS;
	bTransfer_Complete = false;
	
	err = R_SPI_B_WriteRead(pSPIInstance, &byte, rx_buffer, SPI_BYTE_SIZE, pSPIWidth);
	
	assert(FSP_SUCCESS == err);
	
	/* Wait for SPI_EVENT_TRANSFER_COMPLETE callback event. */
	while (false == bTransfer_Complete)
	{
		 R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);
		 if(timeout++ >= 100)
		 {
			 bTransfer_Complete = true;
		 }
	}
	return rx_buffer[0];
}

/**
	Function used to Write one byte over the SPI
*/
uint8_t uCAL_SPI_IO_WriteSingleByte(uint8_t byte)
{
	fsp_err_t err = FSP_SUCCESS;

	bTransfer_Complete = false;
	err = R_SPI_B_Write (pSPIInstance,&byte ,SPI_BYTE_SIZE,pSPIWidth);
	return (uint8_t) err;									 
}

/**
	Function used to desactivate the CS SPI Protocol
*/
void uCAL_SPI_CS_HIGH(void)
{
	R_IOPORT_PinWrite(&g_ioport_ctrl, SPI_CS, BSP_IO_LEVEL_HIGH);
}

/**
	Function used to activate the CS SPI Protocol
*/
void uCAL_SPI_CS_LOW(void)
{
	R_IOPORT_PinWrite(&g_ioport_ctrl, SPI_CS, BSP_IO_LEVEL_LOW);
}