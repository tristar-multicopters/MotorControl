/**
  * @file    comm_config.c
  * @brief   This file instantiate handles required by communication layer.
  *
*/

#include "comm_config.h"
#include "gnr_parameters.h"
#include "co_gnr2_specs.h"

// Handle of the CANOpen node
CO_NODE CONodeGNR;

// Handle of the motor 2. Motor controller 2 is outside of this ganrunner device.
SlaveMotorHandle_t SlaveM2;

UART_Handle_t UART0Handle =
{   
    //.UARTProtocol = UART_LOG_HS,  // for HS Log
    .UARTProtocol = UART_APT,     // for APT
    //.UARTProtocol = UART_KD718,   // for KD718  
    .pUARTInstance = &g_uart9,
};

// SPI communication handle
SPI_Handle_t SPI1Handle =
{
    .bSPI_transfer_complete = false,
    .hSPI_FrameLength = 0x01,
    .pSPI_Instance = &g_spi1_ctrl,
    .pSPI_BitWidth = SPI_BIT_WIDTH_8_BITS,
};

// Flash Memory uCAL Module handle
EFlash_Storage_Handle_t EFlash_Storage_Handle =
{
    .eFlashStorage =
    {
        .eFlash =
        {
            .uCALSPI = &SPI1Handle,
        }
    }
};

APT_Handle_t LCD_APT_handle;
LogHighSpeed_Handle_t LogHS_handle;
KD718_Handle_t LCD_KD718_handle;

//Handle to control the data flash initialisation 
DataFlash_Handle_t DataFlashHandle =
{
	
	.pFlashInstance = &g_flash0,
	.dataFlashOpenFlag = false,

};

