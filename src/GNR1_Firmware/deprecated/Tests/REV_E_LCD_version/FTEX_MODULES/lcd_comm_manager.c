/**
  ******************************************************************************
  * @file    lcd_comm_manager.c
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   Low level module used for managing the communication with compatible
	*          LCD displays.
	*					 Available displays:
	*					 1. Host protocol
  *					 2. Bafang protocol
	*					 ...
	******************************************************************************
	*/

#include "lcd_comm_manager.h"


/**@brief Uart event handler used for notify that a byte has been received or sent
 * 				depending of the type of lcd display (pHandle->lcd_type)
 *
 * @param[in] p_event  : nrf_uart event type
 * @param[in] p_context: pointer towards an LCD_handler type
 */
static void lcd_uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
	LCD_handler_t *pHandle = (LCD_handler_t*)p_context;
	
	switch (p_event->type)
	{
		case NRF_DRV_UART_EVT_RX_DONE:
			break;
		
		case NRF_DRV_UART_EVT_TX_DONE:
			break;
		
		default:
			break;
	}
}

/**@brief Fucntion for initialise the uart driver according to the type of display
 *
 * @param[in] pHandle  : Lcd handle containing the type of screen and uart instance
 * @param[in] uart_config: Uart configuration
 */
void LCD_Init(LCD_handler_t * pHandle, nrf_drv_uart_config_t uart_config)
{
	
}

/**@brief Fucntion for initialise the reception of bytes. Must be used once the uart instance is initialized
 *        and after a received frame has been decoded
 *
 * @param[in] pHandle  : Lcd handle containing the type of screen and uart instance
 * @param[in] rx_buffer: Pointer to the buffer that will receive the data 
 * @param[in] size: number of bytes to receive (normally 1)
 */
void LCD_Receive(LCD_handler_t * pHandle, uint8_t * rx_buffer, uint8_t size)
{
	
}	

/**@brief Fucntion for send bytes. Normally, it should be used for send a response (acknowledge)
 *        once a received frame has been decoded.
 *
 * @param[in] pHandle  : Lcd handle containing the type of screen and uart instance
 * @param[in] tx_buffer: Pointer to the buffer that will contain the data to be sent 
 * @param[in] size: number of bytes to send (normally 1)
 */

void LCD_Send(LCD_handler_t * pHandle, uint8_t * tx_buffer, uint8_t size)
{
	
}

