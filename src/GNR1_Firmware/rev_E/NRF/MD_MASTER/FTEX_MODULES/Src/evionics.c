/**
  ******************************************************************************
  * @file    evionics.c
  * @author  Jorge Andres Polo, FTEX
  * @brief   High level module that controls communication between host (Evionics interface) 				 
  *					 and the GNR	
	******************************************************************************
	*/
	
#include "evionics.h"

/****************** PRIVATE PROTOTYPES AND MEMBERS *******************/
static EVionics_handle_t m_EV_handler;	// EVionics handler
/* Prototypes of private functions */
static void EVionics_TX_IRQ_Handler(void);
static void EVionics_RX_IRQ_Handler(uint8_t rx_data);
/*********************************************************************/

/**@brief Callback used for managing bytes received or sent from the low level layer (euart_manager)
 *
 * @param[in] p_lcd_event: Structure that contains the received byte (or byte to send) and the type of event
 */
static void EVionics_event_handler(eUART_evt_t * p_EVionics_event)
{
	switch(p_EVionics_event->evt_type)
	{
		case EUART_BYTE_RECEIVED:
			EVionics_RX_IRQ_Handler(p_EVionics_event->byte_to_send[0]);
			break;
		
		case EUART_BYTE_SENT:
			EVionics_TX_IRQ_Handler();
			break;
		
		default:
			break;
	}
}

/**@brief Function for build a frame according to the EVionics protocol.
 * @param[in] rx_data: Byte to process into the EV handle
 */
static void EVionics_RX_IRQ_Handler(uint8_t rx_data)
{
	//EV_rx_stage_t current_stage = m_EV_handler.rx_frame.currentStage;
	uint16_t byte_counter = m_EV_handler.rx_frame.ByteCnt;
	
	if(!byte_counter)
	{ // Get the command and request for code bytes
		m_EV_handler.rx_frame.cmd = rx_data;
		m_EV_handler.rx_frame.ByteCnt++;
		eUART_Receive(&m_EV_handler.euart_handler, m_EV_handler.euart_handler.rx_byte);
	}
	else
	{
		uint8_t cmd = m_EV_handler.rx_frame.cmd; // Get the command
		if(cmd == EV_FLASH_GNR) // If we want to update the memory reg table
		{
			uint16_t current_stage = m_EV_handler.rx_frame.currentStage;
			switch(current_stage)
			{
				case EV_FRAME_ID:
				{	
					if(byte_counter == 1) 		// if MSB of code ID received
					{
						m_EV_handler.rx_frame.code |= (rx_data << 8) & 0xFF00; // Get MSB
						m_EV_handler.rx_frame.ByteCnt++;
					}
					else if(byte_counter == 2) // if LSB of code ID received
					{
						m_EV_handler.rx_frame.code |= rx_data & 0xFF; // Get LSB
						m_EV_handler.rx_frame.ByteCnt++;
						m_EV_handler.rx_frame.currentStage = EV_SIZE;
					}
					else
					{
						// TODO: ERROR To code for code
					}	
				}break;
				
				case EV_SIZE:
				{
					if(byte_counter == 3)
					{
						m_EV_handler.rx_frame.size = rx_data; // Get the size of data buffer
						m_EV_handler.rx_frame.ByteCnt++;
						m_EV_handler.rx_frame.currentStage = EV_DATA;
					}
					else
					{
						// TODO: ERROR To code for size
					}	
				}break;
				
				case EV_DATA:
				{
					uint8_t size = m_EV_handler.rx_frame.size;
					uint8_t index_buffer = byte_counter - 4;
					if(index_buffer < size)
					{
						// TODO: Condition for put a 32 bit variable in a buffer element
					}
					else
					{
						m_EV_handler.rx_frame.currentStage = EV_CRC;
					}
					
				}break;
				
				case EV_CRC:
				{
					
				}break;
				
				default:
					break;
			}
		}
		else
		{
			// TO DO... SET OR GET REGISTER
		}
	}
}

/**@brief Function for build a frame for replying to the host (EVionics tool)
 *
 */
static void EVionics_TX_IRQ_Handler(void)
{
	
}

/**@brief Function for process a received frame. This methode has several functions:
	* 			- Configure the motor drive (update the flash memory registers),
	* 			- Monitoring the controller activity (send requested parameters values to EVionics)
	*				- Set local parameters.
*/
void EVionics_frame_process( void )
{
	
}

/**@brief Function for initialise the Uart_0 instance according to EVionics communication
 *				and get the local vehicle instance
 * @param[in] pHandle  : Vehicle handle
 */
void EVionics_init(VCI_Handle_t* pHandle)
{
	/* Initialisation of EV_handler*/
	m_EV_handler.p_VController = pHandle; // Keep a pointer towards the vehicle object
	m_EV_handler.euart_handler.dev_type = EUART_EVIONICS; 				// Attribution of EVionics as dev type
	m_EV_handler.euart_handler.p_evt_handler = EVionics_event_handler; 	// Attribution of event handler
	m_EV_handler.euart_handler.p_uart_inst = UART0_INSTANCE_ADDR; // Attribution of UART0 instance
	m_EV_handler.rx_frame.ByteCnt = 0;
	m_EV_handler.rx_frame.currentStage = EV_FRAME_ID;
	nrf_drv_uart_config_t uart_EVionics_config =
	{                                                     
    .pseltxd            = UART0_TX_PIN,               
    .pselrxd            = UART0_RX_PIN,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = &m_EV_handler.euart_handler, // To be able to recover 
																											 // on the uart event handler on the low level                      
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_115200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	eUART_Init(&m_EV_handler.euart_handler,uart_EVionics_config);
	eUART_Receive(&m_EV_handler.euart_handler, m_EV_handler.euart_handler.rx_byte);	
}
