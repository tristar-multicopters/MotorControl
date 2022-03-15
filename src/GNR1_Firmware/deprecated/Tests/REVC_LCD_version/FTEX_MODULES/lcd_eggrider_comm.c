/**
  ******************************************************************************
  * @file    lcd_eggrider_comm.c
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   High level module that describes Eggrider LCD communication protocol
  *
	******************************************************************************
	*/
	
#include "lcd_eggrider_comm.h"

// Private handler
static EGG_Handle_t m_Egg_handle;
extern osThreadId_t TSK_eUART0_handle; // Task Id for external uart (UART0 instance)

/**@brief Callback used for managing bytes received or sent from the low level layer (lcd_comm_manager)
 *
 * @param[in] p_lcd_event: Structure that contains the received byte (or byte to send) and the type of event
 */
static void LCD_Egg_event_handler(eUART_evt_t * p_lcd_event)
{
	switch(p_lcd_event->evt_type)
	{
		case EUART_BYTE_RECEIVED:
      LCD_EGG_RX_IRQ_Handler(p_lcd_event->byte_to_send[0]);
		  break;
		
		case EUART_BYTE_SENT:
      LCD_EGG_TX_IRQ_Handler();
			break;
		
		default:
			break;
	}
}

/**@brief Function for building a frame specific this the bafang protocol
 * 
 * @param[in] rx_frame: Frame that needs to be decoded. 
 */

void * LCD_EGG_RX_IRQ_Handler(unsigned short rx_data)
{

	 uint8_t ByteCount = m_Egg_handle.rx_frame.ByteCnt;     
   uint8_t ByteReceived = rx_data;
				 
			
		     switch(ByteCount)					 
				 {
					 case 0: 
						 
				     if(ByteReceived == EGG_CMD_READ || ByteReceived == EGG_CMD_WRITE) //Read or write cmd
					   {
					     m_Egg_handle.rx_frame.ByteCnt ++;
					     m_Egg_handle.rx_frame.Code = ByteReceived;
					   }
             else
					   {
					     m_Egg_handle.rx_frame.ByteCnt = 0;
							 
					   }
						 eUART_Receive(&m_Egg_handle.euart_handler, m_Egg_handle.euart_handler.rx_byte);
             break;
					 case 1:
						  m_Egg_handle.rx_frame.Buffer[0] = ByteReceived;
					    m_Egg_handle.rx_frame.ByteCnt ++; 
						    
					    if(m_Egg_handle.rx_frame.Code == EGG_CMD_READ)
							{
								if(m_Egg_handle.rx_frame.Buffer[0] == EGG_STANDARD_READ) //Standard read
								{
								  m_Egg_handle.rx_frame.Size = EGG_STANDARD_READ_SIZE; //Number of bytes expected
								}								
							}
					    else if(m_Egg_handle.rx_frame.Code == EGG_CMD_WRITE)
							{
				        m_Egg_handle.rx_frame.ByteCnt = 0;		
							}
              else if(m_Egg_handle.rx_frame.Code == EGG_CMD_CUSTOM)
							{
							  m_Egg_handle.rx_frame.ByteCnt = 0;
							}
              else
							{
							  m_Egg_handle.rx_frame.ByteCnt = 0; 
							}	
               eUART_Receive(&m_Egg_handle.euart_handler, m_Egg_handle.euart_handler.rx_byte);							
             break;						 
					 default:
						  m_Egg_handle.rx_frame.Buffer[m_Egg_handle.rx_frame.ByteCnt - 1] = ByteReceived;
					    m_Egg_handle.rx_frame.ByteCnt ++;  
					 
								if(m_Egg_handle.rx_frame.Size == m_Egg_handle.rx_frame.ByteCnt)
								{
								  //We have received the complete frame
									m_Egg_handle.rx_frame.ByteCnt = 0;		
                  osThreadFlagsSet(TSK_eUART0_handle, EUART_FLAG); // Notify client a frame has been received									
								}
                else
								{
								  eUART_Receive(&m_Egg_handle.euart_handler, m_Egg_handle.euart_handler.rx_byte);
								}									
						     
            break;						
				 }			

}

void LCD_EGG_TX_IRQ_Handler(void)
{
    uint8_t tx_data;

    if(m_Egg_handle.tx_frame.ByteCnt < m_Egg_handle.tx_frame.Size)
    {
			if(m_Egg_handle.tx_frame.ByteCnt == 0)
			{
			 tx_data = m_Egg_handle.tx_frame.Code;
			}
			else
			{
			 tx_data = m_Egg_handle.tx_frame.Buffer[m_Egg_handle.tx_frame.ByteCnt - 1];
			}
			
			
			m_Egg_handle.tx_frame.ByteCnt ++;
			// Send the data byte 
			eUART_Send(&m_Egg_handle.euart_handler,&tx_data, 1); 
    }
    else
    {	
			//Resume reception of frames			
			m_Egg_handle.tx_frame.ByteCnt = 0;
			eUART_Receive(&m_Egg_handle.euart_handler,m_Egg_handle.euart_handler.rx_byte); //Restart reception
	  }
}

/**@brief Function for decoding a received frame (previously built on the callback function)
 * 
 * @param[in] rx_frame: Frame that needs to be decoded. 
 */

void LCD_EGG_frame_Process(void)
{
	EGG_frame_t replyFrame = {0};
	uint8_t AssistLvl = 0;
	uint32_t toSend;
	uint16_t CheckSum = 0;
	
	//Check checksum
	
	if(LCD_EGG_CRC16(m_Egg_handle.rx_frame) == 0)
	{
	  switch(m_Egg_handle.rx_frame.Code)
	  {
	  	case EGG_CMD_READ:
         
		    if(m_Egg_handle.rx_frame.Buffer[0] == EGG_STANDARD_READ)
				{
				  replyFrame.Code = EGG_CMD_READ;
					replyFrame.Buffer [0] = EGG_STANDARD_READ;
					
					toSend = VC_getBattVoltage(m_Egg_handle.pVController);
					
					toSend = toSend * 100;  //Conversion to 0.01 V / unit from 1 V/ unit
					
					replyFrame.Buffer [1] = 0x13; //toSend & 0xFF; //Write voltage value, every uinit is worth 0.01 V
					replyFrame.Buffer [2] = 0x88; //(toSend >> 8) & 0xFF;
					
					toSend = VC_getMotorSpeedMeas(m_Egg_handle.pVController,M1);
			  
			    toSend = (toSend * 10) / 37;
					
				  replyFrame.Buffer [3] = 0x04;//toSend & 0x00FF; //Write speed value, every uinit is worth 0.01 Km/h
					replyFrame.Buffer [4] = 0xE2;//(toSend >> 8) & 0x00FF;
					
					replyFrame.Buffer [5] = 0x13; //Write Current value, every uinit is worth 0.01 A (N/A)
					replyFrame.Buffer [6] = 0x88;
					
					replyFrame.Buffer [7] = 0x64; //Write Battery % (0-100) (N/A)
					
					replyFrame.Buffer [8] = 0xFF; //Write pedal cadence     (N/A)
					
					replyFrame.Buffer [9] = 0xFF; //Write Human torque      (N/A)
					
					toSend = VC_getMotorTemp(m_Egg_handle.pVController,M1);
				
  				toSend += 50;
					
					replyFrame.Buffer[10] = 0xFF; //toSend; //Write motor temp with 0 = -50 celcius
					
					toSend = VC_getInverterHeatsinkTemp(m_Egg_handle.pVController,M1);
					
					toSend += 50;
					
					replyFrame.Buffer[11] = 0xFF; //toSend; //Write controller temp with 0 = -50 celcius
					
					replyFrame.Buffer[12] = 0xFF;   //Write Battery temp with 0 = -50 celcius  (N/A)
					
					replyFrame.Buffer[13] = 0x00;   //Write error code 
					
				  replyFrame.Buffer[14] = 0xFF;   //Write misc flags (front light, rear light ,etc) (N/A)

          replyFrame.Buffer[15] = 0xFF;   //Spare byte
          
					
					replyFrame.Size = 17; //Set the frame size for the CRC calculation
					
					//Calculate CRC16 checksum
					CheckSum = LCD_EGG_CRC16(replyFrame);
					//CRC should be 35969 = 0x8C81 
					
					replyFrame.Buffer[16] = (CheckSum >> 8)& 0x00FF; //Write checksum
					replyFrame.Buffer[17] = (CheckSum) & 0x00FF;
				  
					
					replyFrame.Size = replyFrame.Size + 2; //Add the two CRC bytes to the frame size
				}
		
		
			
	  	break; //end case EGG_CMD_READ
		
	  	case EGG_CMD_WRITE:
  	
  		break;//end case EGG_CMD_WRITE
		
		  default: // TODO: Rise an error
	  		replyFrame.Size = 0;
	  	  eUART_Receive(&m_Egg_handle.euart_handler, m_Egg_handle.euart_handler.rx_byte);
	  		break;
  	}
 
	}//checksum check
	
	if(replyFrame.Size > 0)
	{
		replyFrame.ByteCnt = 0;
		
		m_Egg_handle.tx_frame = replyFrame; 
     		
		LCD_EGG_TX_IRQ_Handler();
	}	
	else 
	{
		eUART_Receive(&m_Egg_handle.euart_handler, m_Egg_handle.euart_handler.rx_byte);
	}
}

/**@brief Function for initializing the LCD module with the EggRider protocol
 * 
 * @param[in] pHandle: Handle for vehicle controller 
 */
void LCD_EGG_init(VC_Handle_t * pHandle)
{
		m_Egg_handle.pVController = pHandle;  					 								  // Pointer to VController
		m_Egg_handle.euart_handler.dev_type = EUART_EGG;							    // Attribution of LCD type
	  m_Egg_handle.euart_handler.p_evt_handler = LCD_Egg_event_handler; // Attribution of callback function
		m_Egg_handle.euart_handler.p_uart_inst = UART0_INSTANCE_ADDR;			// Attribution of UART0 adresse
	
		nrf_drv_uart_config_t uart_drive_config =
	  {                                                     
      .pseltxd            = UART0_TX_PIN,               
      .pselrxd            = UART0_RX_PIN,              	
      .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
      .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
      .p_context          = &m_Egg_handle.euart_handler, // To be able to recover on the uart event handler on the low level  	                  
      .hwfc               = NRF_UART_HWFC_DISABLED,       
      .parity             = NRF_UART_PARITY_EXCLUDED,     
      .baudrate           = NRF_UART_BAUDRATE_9600, 		
      .interrupt_priority = 2,                  					
      NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	  };
	
	eUART_Init(&m_Egg_handle.euart_handler,uart_drive_config);
	eUART_Receive(&m_Egg_handle.euart_handler, m_Egg_handle.euart_handler.rx_byte);	
}
/* Checksum calculation function
   Used to make the 16 bits chesum when sending a frame or
   to verify a received frame to see if it is valid.
*/
uint16_t LCD_EGG_CRC16(EGG_frame_t frame)
{
  uint16_t crc = 0xFFFF;
  uint16_t x;
  uint8_t data = 0; 
	
   for(int i = 0; i < frame.Size; i ++)   
      {
				 if(i == 0) //first byte received is the code
				 {
				   data = frame.Code;
				 }
				else //the buffer contaisn the rest
				 {
				   data = frame.Buffer[i-1];
				 }
         x = ((crc >> 8) ^ data) & 0xFF;
         x ^= (x >> 4);
         crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;				
      }		
  return(crc);

}