/**
  ******************************************************************************
  * @file    euart_manager.c
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   Low level module used for managing the communication over UART0 
	*					 (used normally for LCD displays).
	******************************************************************************
	*/

#include "euart_manager.h"

osThreadId_t TSK_eUART0_handle; // Task Id for external uart (UART0 instance)

/**@brief Uart event handler used for notify that a byte has been received or sent
 * 				depending of the type of lcd display (pHandle->lcd_type)
 *
 * @param[in] p_event  : nrf_uart event type
 * @param[in] p_context: pointer towards an LCD_handler type
 */
static void euart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
	eUART_handler_t *pHandle = (eUART_handler_t*)p_context;
	switch (p_event->type)
	{
		case NRF_DRV_UART_EVT_RX_DONE: //Byte received
		{
			eUART_evt_t euart_evt = {EUART_BYTE_RECEIVED, pHandle->rx_byte[0]};
			pHandle->p_evt_handler(&euart_evt);
		}break;
		
		case NRF_DRV_UART_EVT_TX_DONE: //Byte sent
		{
			eUART_evt_t euart_evt = {EUART_BYTE_SENT, NULL};
			pHandle->p_evt_handler(&euart_evt);
		}break;
		
		case NRF_DRV_UART_EVT_ERROR: //Error occured
			pHandle->hError = true;
			break;
		
		default:
			break;
	}
}

/**@brief Fucntion to initialise the uart driver according to the type of display
 *
 * @param[in] pHandle    : Lcd handle containing the type of screen and uart instance
 * @param[in] uart_config: Uart configuration
 */
void eUART_Init(eUART_handler_t * pHandle, nrf_drv_uart_config_t uart_config)
{	
	// Verify if instance was already initialised
	if(nrf_drv_uart_init(pHandle->p_uart_inst, &uart_config, euart_event_handler) != NRF_SUCCESS )
	{
		nrf_drv_uart_uninit (pHandle->p_uart_inst);                                //Try to Uninitialise before reinitialising
		
		if(nrf_drv_uart_init(pHandle->p_uart_inst, &uart_config, euart_event_handler) == NRF_SUCCESS)
		{
		  nrf_drv_uart_rx_enable(pHandle->p_uart_inst); // Enable the reception if uart reinitialization was succesful
		}
	}
	else
	{
		nrf_drv_uart_rx_enable(pHandle->p_uart_inst); // Enable the reception if uart initialization was succesful
	}
}

/**@brief Fucntion for sending bytes. Normally, it should be used to send a response (acknowledge)
 *        once a received frame has been decoded.
 *
 * @param[in] pHandle  : Lcd handle containing the type of screen and uart instance
 * @param[in] tx_buffer: Pointer to the buffer that will contain the data to be sent 
 * @param[in] size: number of bytes to send (normally 1)
 */
void eUART_Send(eUART_handler_t * pHandle, uint8_t * tx_buffer, uint8_t size)
{
	nrf_drv_uart_tx(pHandle->p_uart_inst, tx_buffer, size);
}	

/**@brief Fucntion to initialise the reception of bytes. Must be used once the uart instance is initialized
 *        and after a received frame has been decoded
 *
 * @param[in] pHandle  : Lcd handle containing the type of screen and uart instance
 * @param[in] rx_buffer: Pointer to the buffer that will receive the data 
 */
void eUART_Receive(eUART_handler_t * pHandle, uint8_t * rx_buffer)
{
	nrf_drv_uart_rx(pHandle->p_uart_inst, rx_buffer, 1);
}
