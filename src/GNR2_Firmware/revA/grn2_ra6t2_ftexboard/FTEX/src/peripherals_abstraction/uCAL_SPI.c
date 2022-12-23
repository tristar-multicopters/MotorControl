/**
  ******************************************************************************
  * @file    uCAL_SPI.c
  * @author  FTEX inc
  * @brief   uCAL SPI communication protocol module using General functions
  ******************************************************************************
*/

// =============================== Includes ================================= //
#include "uCAL_SPI.h"
#include "ASSERT_FTEX.h"

// ==================== Public function prototypes ======================== //

/**
	Function used to initialize the SPI Protocol
*/
void uCAL_SPI_Init(SPI_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL); 
}	

/**
	Function used to read/write byte via SPI Protocol
*/
uint8_t uCAL_SPI_IO_WriteByte(SPI_Handle_t * pHandle, uint8_t byte)
{
	fsp_err_t err = FSP_SUCCESS;
	
	pHandle->bSPI_transfer_complete = false;
	err = R_SPI_B_WriteRead(pHandle->pSPI_Instance, &byte, pHandle->rx_buffer, pHandle->hSPI_FrameLength, pHandle->pSPI_BitWidth);

	assert(FSP_SUCCESS == err);
	
	/* Wait for SPI_EVENT_TRANSFER_COMPLETE callback event. */
	while (false == pHandle->bSPI_transfer_complete)
	{
		err = false;
	}	
	return pHandle->rx_buffer[0];
}

/**
	Function used to Write one byte over the SPI
*/
uint8_t uCAL_SPI_IO_WriteSingleByte(SPI_Handle_t * pHandle, uint8_t byte)
{
	fsp_err_t err = FSP_SUCCESS;
	
	pHandle->bSPI_transfer_complete = false;
	err = R_SPI_B_Write (pHandle->pSPI_Instance,&byte, pHandle->hSPI_FrameLength , pHandle->pSPI_BitWidth);
	return (uint8_t) err;									 
}

/**
	Function used to Read one byte over the SPI
*/
uint8_t uCAL_SPI_IO_ReadSingleByte(SPI_Handle_t * pHandle)
{
	uint8_t data;
	
	pHandle->bSPI_transfer_complete = false;
	R_SPI_B_Read(pHandle->pSPI_Instance, &data, pHandle->hSPI_FrameLength, pHandle->pSPI_BitWidth);
    	
    return data;
}

/**
	Function used to desactivate the CS SPI Protocol
*/
void uCAL_SPI_Disable(void)
{
	R_IOPORT_PinWrite(&g_ioport_ctrl, SPI_CS, BSP_IO_LEVEL_HIGH);
}

/**
	Function used to activate the CS SPI Protocol
*/
void uCAL_SPI_Enable(void)
{
	R_IOPORT_PinWrite(&g_ioport_ctrl, SPI_CS, BSP_IO_LEVEL_LOW);
}