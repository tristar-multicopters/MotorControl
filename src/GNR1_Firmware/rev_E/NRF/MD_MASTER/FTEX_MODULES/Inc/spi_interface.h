/**
  ******************************************************************************
  * @file    spi_interface.h
	* @author  Kian Gheitasi, FTEX
  * @author  Sami Bouzid, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   This module provides a general framwork to communicate with devices using SPI
  *
	******************************************************************************
	*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_INTERFACE_H
#define __SPI_INTERFACE_H

/* Includes ------------------------------------------------------------------*/

#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "nrf_queue.h"

//#define SPI_COMM_DEBUG
#ifdef SPI_COMM_DEBUG
#include "bsp.h"
#endif

#define SPI_MAX_BUFFER_SIZE    		16
#define MAX_NUMBER_OF_DEVICES  		5
#define SPI_QUEUE_SIZE  					32

typedef enum
{
	SPI_NO_ERROR,
	SPI_TX_ERROR,
	SPI_QUEUE_EMPTY,
	SPI_QUEUE_FULL,
} SPI_error_t;

typedef struct 
{
	uint8_t device_id;
	uint8_t tx_buffer[SPI_MAX_BUFFER_SIZE];
	uint8_t rx_buffer[SPI_MAX_BUFFER_SIZE];
	uint8_t tx_len;
	uint8_t rx_len;
	
} SPI_Transfer_t;

typedef void (* spi_callback_end_t)(void);

typedef struct {
	spi_callback_end_t pCallback_end;
	uint32_t ss_pin;
	uint8_t device_id;
	SPI_Transfer_t current_transfert; // transfert where the rx_data will be captured
	
} SPIdevice_t;

typedef struct {
	nrf_drv_spi_t* p_spi_inst;
	nrf_drv_spi_config_t spi_config;
	const nrf_queue_t* queue;
	
	SPIdevice_t* aDevices[MAX_NUMBER_OF_DEVICES];
	uint8_t current_id;
	volatile bool bIsTransferOngoing;
	uint8_t nb_of_dev;
	SPI_error_t hError;
	
} SPI_Handle_t;


/*********************************************************************************************************
  BEGIN FUNCTIONS SPI
*********************************************************************************************************/

void SPI_Transfer_IT(SPI_Handle_t *p_Handle, SPI_Transfer_t* transfer);
void SPI_Transfer_Wait(SPI_Handle_t *p_Handle, SPI_Transfer_t* transfer);
	
ret_code_t SPI_Init(SPI_Handle_t *spi_handle);

uint8_t SPI_AddDevice(SPI_Handle_t *p_Handle, SPIdevice_t* pDevice);



#endif /* __SPI_INTERFACE_H */
