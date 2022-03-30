/**
  ******************************************************************************
  * @file    ev_config_tool.c
  * @author  Jorge Andres Polo, FTEX
  * @brief   High level module that controls communication between host (Evionics interface) 				 
  *					 and the GNR	
	******************************************************************************
	*/
	
#include "ev_config_tool.h"

/****************** PRIVATE PROTOTYPES AND MEMBERS *******************/
static EVCT_handle_t m_EVCT_handler = {0};	// EVCT handler
/* Prototypes of private functions */
static void EVCT_init_handler(EVCT_handle_t * pHandle);
static void EVCT_TX_IRQ_Handler(void);
static void EVCT_RX_IRQ_Handler(uint8_t rx_data);
static int32_t EVCT_recoverInt32(uint16_t index);
static uint16_t EVCT_CRC16( uint8_t * buffer, uint16_t size );
static void EVCT_send_data(uint8_t ack_code, uint16_t code);
/*********************************************************************/

extern osThreadId_t TSK_eUART0_handle; // eUART task identifier
extern osThreadId_t TSK_STRG_handle;   // Storage management task identifier
/**@brief Callback used for managing bytes received or sent from the low level layer (euart_manager)
 *
 * @param[in] p_lcd_event: Structure that contains the received byte (or byte to send) and the type of event
 */
static void EVCT_event_handler(eUART_evt_t * p_EVCT_event)
{
	switch(p_EVCT_event->evt_type)
	{
		case EUART_BYTE_RECEIVED:
			EVCT_RX_IRQ_Handler(p_EVCT_event->byte_to_send[0]);
			break;
		
		case EUART_BYTE_SENT:
			EVCT_TX_IRQ_Handler();
			break;
		
		default:
			break;
	}
}

/**@brief Function for build a frame according to the EVCT protocol.
 * @param[in] rx_data: Byte to process into the EV handle
 */
static void EVCT_RX_IRQ_Handler(uint8_t rx_data)
{
	//EV_rx_stage_t current_stage = m_EVCT_handler.rx_frame.currentStage;
	uint16_t byte_counter = m_EVCT_handler.rx_frame.ByteCnt;
	
	if(!byte_counter)
	{ // Get the command and request for code bytes
		m_EVCT_handler.rx_frame.cmd = rx_data;
		m_EVCT_handler.rx_frame.ByteCnt++; // Increase byte counter
		
		eUART_Receive(&m_EVCT_handler.euart_handler, m_EVCT_handler.euart_handler.rx_byte);
	}
	else
	{
		uint8_t cmd = m_EVCT_handler.rx_frame.cmd; // Get the command
		if(cmd == EV_LOAD_PARAMS) // If we want to update the memory reg table
		{
			uint16_t current_stage = m_EVCT_handler.rx_frame.currentStage;
			switch(current_stage)
			{
				case EV_FRAME_ID:
				{
					if(byte_counter == 1) 													// if MSB of code ID received
					{
						m_EVCT_handler.rx_frame.code |= (rx_data << 8) & 0xFF00; // Get MSB
						m_EVCT_handler.rx_frame.ByteCnt++;
					}
					else if(byte_counter == 2) 											// if LSB of code ID received
					{
						m_EVCT_handler.rx_frame.code |= rx_data & 0xFF; // Get LSB
						m_EVCT_handler.rx_frame.ByteCnt++;		 					// Increment of bytes counter
						m_EVCT_handler.rx_frame.currentStage = EV_SIZE;
					}
					else
					{
						// TODO: ERROR To code for code
					}
					eUART_Receive(&m_EVCT_handler.euart_handler, m_EVCT_handler.euart_handler.rx_byte);	// Request a new byte				
				}break;
				
				case EV_SIZE:
				{
					if(byte_counter == 3)
					{
						m_EVCT_handler.rx_frame.size = rx_data; // Get the size of data buffer that should be 255
						m_EVCT_handler.rx_frame.ByteCnt++;		  // Increment of bytes counter
						m_EVCT_handler.rx_frame.currentStage = EV_DATA_CRC;
					}
					else
					{
						// TODO: ERROR To code for size
					}
					eUART_Receive(&m_EVCT_handler.euart_handler, m_EVCT_handler.euart_handler.rx_byte);					
				}break;
				
				case EV_DATA_CRC:
				{ // Get the data and CRC code
					uint8_t size = m_EVCT_handler.rx_frame.size;
					uint16_t index_buffer = byte_counter - 4;
					if(index_buffer < size) // Save data and first byte of CRC in local table
					{
						// Put received data in the local buffer
						m_EVCT_handler.rx_frame.buffer[index_buffer] = rx_data;
						m_EVCT_handler.rx_frame.ByteCnt++; // Increment of bytes counter
						eUART_Receive(&m_EVCT_handler.euart_handler, m_EVCT_handler.euart_handler.rx_byte); //request a new byte
					}
					else
					{ // Reset the state machine and notify the eUART task a frame has been built
						m_EVCT_handler.rx_frame.buffer[index_buffer] = rx_data; // Get second byte of CRC
						if(index_buffer == size + CRC_SIZE)
						{	
							osThreadFlagsSet(TSK_eUART0_handle, EUART_FLAG);		  // Notify eUART task
						}
						else
						{
							m_EVCT_handler.rx_frame.ByteCnt++;											// Increment of bytes counter
							eUART_Receive(&m_EVCT_handler.euart_handler, m_EVCT_handler.euart_handler.rx_byte); //request LSB of CRC code
						}
					}
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

/**@brief Function for build a frame for replying to the host (EVCT tool)
 *
 */
static void EVCT_TX_IRQ_Handler(void)
{
	uint16_t rx_cmd 		= m_EVCT_handler.rx_frame.cmd;
	uint16_t tx_counter = m_EVCT_handler.tx_frame.ByteCnt;
	uint8_t tx_size 		= m_EVCT_handler.tx_frame.size;
	switch(rx_cmd)
	{
		case EV_LOAD_PARAMS:
		{
			if(tx_counter < tx_size + CRC_SIZE + 1) // + 1 is for the size byte to send
			{
				m_EVCT_handler.tx_frame.ByteCnt++; // Increment tx counter
				if(!tx_counter)
					// Send the size of data packet
					eUART_Send(&m_EVCT_handler.euart_handler, &m_EVCT_handler.tx_frame.size, 1);
				else
					// Send the data (included CRC16 code)
					eUART_Send(&m_EVCT_handler.euart_handler, &m_EVCT_handler.tx_frame.buffer[tx_counter-1], 1);
			}
			else
			{
				//Restart the reception and the rx_buffer and tx_buffer of the handle
				memset(&m_EVCT_handler.rx_frame,0,sizeof(m_EVCT_handler.rx_frame));
				memset(&m_EVCT_handler.tx_frame,0,sizeof(m_EVCT_handler.tx_frame));
				eUART_Receive(&m_EVCT_handler.euart_handler, m_EVCT_handler.euart_handler.rx_byte);
			}
		}break;
	}
}

/**@brief Function for process a received frame. This methode has several functions:
	* 			- Configure the motor drive (update the flash memory registers),
	* 			- Monitoring the controller activity (send requested parameters values to EVCT)
	*				- Set local parameters.
*/
void EVCT_frame_process( void )
{
	uint8_t cmd 	= m_EVCT_handler.rx_frame.cmd;
	uint8_t size 	= m_EVCT_handler.rx_frame.size;
	EV_regSectors_t code = (EV_regSectors_t)m_EVCT_handler.rx_frame.code;
	bool isError = false;
	// Verify CRC
	if(!EVCT_CRC16(m_EVCT_handler.rx_frame.buffer,size+CRC_SIZE+1)) // size + 1 for having 256 bytes
	{
		switch(cmd)
		{
			case EV_LOAD_PARAMS:
			{
				uint16_t iter_by_4 = 0;   // Iterator by 4 for getting the int32_t parameter by extracting bytes from rx_frame buffer 
				for(uint8_t i = 0; i < size ; i++) // i: iterator for local flash register table
				{ // Restore int 32 value from 4 bytes of the buffer
					int32_t value = EVCT_recoverInt32(iter_by_4);
					// Put the value in the flash register table according to the received sector code.
					STRG_setParam(code+i,value);
					iter_by_4 += 4;
				}
				// If all parameters are been acquired
				if(code == MANUF_SECT_4)
				{
					// Notify task for overwritting the flash memory with the received params
					osThreadFlagsSet(TSK_STRG_handle, STRG_FLAG); // Notify Storage task
					// Hold on until the memory is free
					while(STRG_isBusy());
					m_EVCT_handler.rx_frame.currentStage = EV_FRAME_ID;	    // Restart state machine for next updates
				}
				else
					EVCT_send_data((uint8_t)(ACK_NOERROR), code);
			}break;
			/*	TO DO FOR MONITORING MODE */
			case EV_SET_REG:
			case EV_GET_REG:
				break;
			/*********************************/
			default:
				/* TO DO: Error to code...*/
				isError = true;
				break;
		}
	}
	
	else
	{
		// Manage CRC_code error
		isError = true;
	}
	
	if(!isError)
	{
		if(m_EVCT_handler.rx_frame.cmd == EV_LOAD_PARAMS)
		{ 
			// Build the message response with no error
			EVCT_send_data((uint8_t)ACK_NOERROR, code);
		}
	}
	else
	{// Build the message response with error
		EVCT_send_data((uint8_t)ACK_ERROR, code);
	}
	
//	else
//	{
//		// TODO: Bad CRC code managing...
//	}
}

/**@brief Function for initialise the Uart_0 instance according to EVCT communication
 *				and get the local vehicle instance
 * @param[in] pHandle  : Vehicle handle
 */
void EVCT_init(VCI_Handle_t* pHandle)
{
	/* Initialisation of EV_handler*/
	EVCT_init_handler(&m_EVCT_handler);
	m_EVCT_handler.p_VController = pHandle; // Keep a pointer towards the vehicle object
	nrf_drv_uart_config_t uart_EVCT_config =
	{                                                     
    .pseltxd            = UART0_TX_PIN,               
    .pselrxd            = UART0_RX_PIN,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = &m_EVCT_handler.euart_handler, // To be able to recover 
																											 // on the uart event handler on the low level                      
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_115200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	eUART_Init(&m_EVCT_handler.euart_handler,uart_EVCT_config);
	eUART_Receive(&m_EVCT_handler.euart_handler, m_EVCT_handler.euart_handler.rx_byte);	
}

/**@brief Function for initializing the handler of the Evionics module
 * @param[in] pHandle  : Evionics handle
 */
static void EVCT_init_handler(EVCT_handle_t *pHandle)
{
	pHandle->euart_handler.dev_type = EUART_EVIONICS; 						  // Attribution of EVCT as dev type
	pHandle->euart_handler.p_evt_handler = EVCT_event_handler; 	// Attribution of event handler
	pHandle->euart_handler.p_uart_inst = UART0_INSTANCE_ADDR; 			// Attribution of UART0 instance
	pHandle->rx_frame.currentStage = EV_FRAME_ID;										// Init at start parameters section
}

/**@brief Checksum calculation function
 * 				Used to make the 16 bits chesum when sending a frame or
 *  		  to verify a received frame to see if it is valid (it should give 0).
 * @param[in] buffer  : Data buffer
 * @param[in] size    : Buffer size
 */
static uint16_t EVCT_CRC16( uint8_t * buffer, uint16_t size )
{
  uint16_t crc = 0xFFFF;
  uint16_t x;
  uint8_t data = 0;
  for(int i = 0; i < size; i ++)
	{
		 data = buffer[i];
		 x = ((crc >> 8) ^ data) & 0xFF;
		 x ^= (x >> 4);
		 crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
	}
  return(crc);
}

/**@brief Function for recoving a int32 value from four bytes received 
					in the rx_frame buffer of the Evionics handler. 
 * @param[in] index: Actual index of the rx_frame buffer
 */
static int32_t EVCT_recoverInt32(uint16_t index)
{
	return m_EVCT_handler.rx_frame.buffer[index]   << 24 |  // MSB of value
			 	 m_EVCT_handler.rx_frame.buffer[index+1] << 16 |
				 m_EVCT_handler.rx_frame.buffer[index+2] << 8  |
		     m_EVCT_handler.rx_frame.buffer[index+3];				  // LSB of value
}

/**@brief Function for generating a reply message to the host
 * 				
 * @param[in] ack_code  : Acknowledge code (ACK_ERROR or ACK_NOERROR)
 * @param[in] code      : Parameter section code
 */
static void EVCT_send_data(uint8_t ack_code, uint16_t code)
{
	m_EVCT_handler.tx_frame.tx_code = ack_code;  	 // add Header of frame
	if(ack_code == ACK_NOERROR)
	{
			/* Build of data transmission buffer */
			m_EVCT_handler.tx_frame.size = 2;
			m_EVCT_handler.tx_frame.buffer[0] = (code >> 8) & 0xFF;		 // MSB of Section code
			m_EVCT_handler.tx_frame.buffer[1] = code & 0xFF;		 			 // LSB of Section code
			// Add CRC
			uint16_t tx_CRC = EVCT_CRC16(m_EVCT_handler.tx_frame.buffer, m_EVCT_handler.tx_frame.size);
			m_EVCT_handler.tx_frame.buffer[2] = (tx_CRC >> 8) & 0xFF;	 // MSB of CRC
			m_EVCT_handler.tx_frame.buffer[3] = tx_CRC & 0xFF;				 // LSB of CRC
			// Send the message via UART
			eUART_Send(&m_EVCT_handler.euart_handler, &m_EVCT_handler.tx_frame.tx_code, 1);
	}
}
