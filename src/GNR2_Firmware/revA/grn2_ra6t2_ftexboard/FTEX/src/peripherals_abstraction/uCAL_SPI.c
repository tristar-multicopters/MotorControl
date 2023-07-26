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
    uint16_t spiInterrupTimeOut = 0;
	
    pHandle->bSPI_transfer_complete = false;
    pHandle->bSPI_transfer_failed = false;
    err = R_SPI_B_WriteRead(pHandle->pSPI_Instance, &byte, pHandle->rx_buffer, pHandle->hSPI_FrameLength, pHandle->pSPI_BitWidth);

    assert(FSP_SUCCESS == err);
	
    /* Wait for SPI_EVENT_TRANSFER_COMPLETE callback event. */
    while (false == pHandle->bSPI_transfer_complete)
    {
        //delay 1us to count timeout.
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);
        
        //increment the timeout.
        spiInterrupTimeOut++;
        
        //wait max 500us to spi interruption
        if (spiInterrupTimeOut >= SPI_INTERRUPTION_TIMEOUT_500US)
        {
            //set the fail flag to true.
            pHandle->bSPI_transfer_failed = true;
            
            //break the loop
            break;
        }
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
    
    //add a delay of 100us to give enough time to the CS live
    //goes high.
    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MICROSECONDS);
}

/**
	Function used to activate the CS SPI Protocol
*/
void uCAL_SPI_Enable(void)
{
	R_IOPORT_PinWrite(&g_ioport_ctrl, SPI_CS, BSP_IO_LEVEL_LOW);
    
    //add a delay of 200us to give enough time to the CS live
    //goes high.
    R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MICROSECONDS);
}

/**
	Function used get the flag that indicate if the spi operation failed.
*/
bool uCAL_SPI_IO_GetSpiFailedFlag(SPI_Handle_t * pHandle)
{
    return  pHandle->bSPI_transfer_failed;
}