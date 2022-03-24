/**
  ******************************************************************************
  * @file    spi_interface.c
  * @author  Sami Bouzid, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   This module provides a general framwork to communicate with devices using SPI
  *
	******************************************************************************
	*/

#include "spi_interface.h"

/* Private variables ---------------------------------------------------------*/

static SPI_Handle_t *m_pSPI_handle;						/**< Pointer to the SPI global structure */

NRF_QUEUE_DEF(SPI_Transfer_t, m_transfert_queue, 16, NRF_QUEUE_MODE_NO_OVERFLOW);

/*------------------------------------ SPI Functions ---------------------------------------*/

//TO CHECK...
static void Start_Transfer(void)
{
	ret_code_t res;
	
	SPI_Transfer_t xfer;
	//Only for recovering the transfer ID
	res = nrf_queue_peek(m_pSPI_handle->queue, &xfer);
	m_pSPI_handle->current_id = xfer.device_id;
	
	res = nrf_queue_pop(m_pSPI_handle->queue, &m_pSPI_handle->aDevices[m_pSPI_handle->current_id]->current_transfert);
	if(res != NRF_SUCCESS)
		m_pSPI_handle->hError = SPI_QUEUE_EMPTY;
	
	uint32_t pin = m_pSPI_handle->aDevices[m_pSPI_handle->current_id]->ss_pin;
	nrf_gpio_pin_clear(pin);

	res = nrf_drv_spi_transfer(m_pSPI_handle->p_spi_inst, 
												m_pSPI_handle->aDevices[m_pSPI_handle->current_id]->current_transfert.tx_buffer, 
												m_pSPI_handle->aDevices[m_pSPI_handle->current_id]->current_transfert.tx_len,
												m_pSPI_handle->aDevices[m_pSPI_handle->current_id]->current_transfert.rx_buffer,
												m_pSPI_handle->aDevices[m_pSPI_handle->current_id]->current_transfert.rx_len);
}


void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	uint32_t pin = m_pSPI_handle->aDevices[m_pSPI_handle->current_id]->ss_pin;
	nrf_gpio_pin_set(pin);
	
	m_pSPI_handle->aDevices[m_pSPI_handle->current_id]->pCallback_end();
	
	if ( nrf_queue_is_empty(m_pSPI_handle->queue) )
	{
		m_pSPI_handle->bIsTransferOngoing = false;
	}
	else
	{
			Start_Transfer();
	}
}


void SPI_Transfer_IT(SPI_Handle_t *p_Handle, SPI_Transfer_t* transfer)
{
	ret_code_t res;
	
	p_Handle->aDevices[transfer->device_id]->current_transfert = *transfer;
	
	res = nrf_queue_push(p_Handle->queue, transfer);
	
	if(res == NRF_ERROR_NO_MEM)
	{
		p_Handle->hError = SPI_QUEUE_FULL;
		return;
	}
	
	if (p_Handle->bIsTransferOngoing == false)
	{
		p_Handle->bIsTransferOngoing = true;
		Start_Transfer();
	}
}

void SPI_Transfer_Wait(SPI_Handle_t *p_Handle, SPI_Transfer_t* transfer)
{
	ret_code_t res;
	
	p_Handle->aDevices[transfer->device_id]->current_transfert = *transfer;
	
	res = nrf_queue_push(p_Handle->queue, transfer);
	
	if(res == NRF_ERROR_NO_MEM)
	{
		p_Handle->hError = SPI_QUEUE_FULL;
		return;
	}
			
	while (p_Handle->bIsTransferOngoing == true);
	p_Handle->bIsTransferOngoing = true;
	Start_Transfer();
	while (p_Handle->bIsTransferOngoing == true);
}

uint8_t SPI_AddDevice(SPI_Handle_t *p_Handle, SPIdevice_t* pDevice)
{
	uint8_t device_id = p_Handle->nb_of_dev;
	p_Handle->aDevices[device_id] = pDevice;
	p_Handle->nb_of_dev++;
	
	return device_id;
}

void SPI_Init(SPI_Handle_t *p_Handle)
{		
	m_pSPI_handle = p_Handle;
	p_Handle->hError = SPI_NO_ERROR;
	
	p_Handle->queue = &m_transfert_queue;
	p_Handle->bIsTransferOngoing = false;
	p_Handle->current_id = 0;

	ret_code_t res = nrf_drv_spi_init(p_Handle->p_spi_inst, &p_Handle->spi_config, spi_event_handler, NULL);
	
	//todo: handle returned result

	#ifdef SPI_COMM_DEBUG
	bsp_board_init(BSP_INIT_LEDS);
	#endif
}
