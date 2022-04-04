/**
  ******************************************************************************
  * @file    lcd_ftex_comm.c
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   High level module that describes FTEX LCD communication protocol
  *
	******************************************************************************
	*/
	
#include "lcd_ftex_comm.h"

// Private handler
FTEX_Handle_t m_FTEX_handle;
extern osThreadId_t TSK_eUART0_handle; // Task Id for external uart (UART0 instance)

/**@brief Callback used for managing bytes received or sent from the low level layer (euart_manager)
 *
 * @param[in] p_lcd_event: Structure that contains the received byte (or byte to send) and the type of event
 */
static void LCD_FTEX_event_handler(eUART_evt_t * p_lcd_event)
{
	switch(p_lcd_event->evt_type)
	{
		case EUART_BYTE_RECEIVED:
      LCD_FTEX_RX_IRQ_Handler(p_lcd_event->byte_to_send[0]);
		  break;
		
		case EUART_BYTE_SENT:
      LCD_FTEX_TX_IRQ_Handler();
			break;
	}
}

/**@brief Function for building a frame specific to the FTEX protocol
 * 
 * @param[in] rx_dara: latest byte that has been received 
 */
void LCD_FTEX_RX_IRQ_Handler(unsigned short rx_data)
{

	 uint8_t ByteCount = m_FTEX_handle.rx_frame.ByteCnt;     
   uint8_t ByteReceived = rx_data;
				 			
	 switch(ByteCount)					 
	 {
	  case 0: //Check if first byte is a valid frame type
						 
		  if(ByteReceived == FTEX_FRAME_READ || ByteReceived == FTEX_FRAME_WRITE || ByteReceived == FTEX_FRAME_CUSTOM) //Read or write cmd
		  {
		    m_FTEX_handle.rx_frame.ByteCnt ++;
		    m_FTEX_handle.rx_frame.Code = ByteReceived;
		  }
      else
		  {
		    m_FTEX_handle.rx_frame.ByteCnt = 0;							 
		  }
		  eUART_Receive(&m_FTEX_handle.euart_handler, m_FTEX_handle.euart_handler.rx_byte);
      break;
		case 1: //Check if second byte is a valid frame subtype
		  m_FTEX_handle.rx_frame.Buffer[0] = ByteReceived;
		  m_FTEX_handle.rx_frame.ByteCnt ++; 
						    
		  if(m_FTEX_handle.rx_frame.Buffer[0] == FTEX_F_SUBTYPE_01)
			{
			  if(m_FTEX_handle.rx_frame.Code == FTEX_FRAME_READ)
				{
				   m_FTEX_handle.rx_frame.Size = 4;
				}
				else if(m_FTEX_handle.rx_frame.Code == FTEX_FRAME_WRITE)
				{
							
				}
				else if(m_FTEX_handle.rx_frame.Code == FTEX_FRAME_CUSTOM)
		  	{
								
				}	
			}
			else
			{
			 	m_FTEX_handle.rx_frame.ByteCnt = 0;
			}
      eUART_Receive(&m_FTEX_handle.euart_handler, m_FTEX_handle.euart_handler.rx_byte);							
      break;						 
		default:
		  m_FTEX_handle.rx_frame.Buffer[m_FTEX_handle.rx_frame.ByteCnt - 1] = ByteReceived;
		  m_FTEX_handle.rx_frame.ByteCnt ++;  
					 
			if(m_FTEX_handle.rx_frame.Size == m_FTEX_handle.rx_frame.ByteCnt)
		  {
		    //We have received the complete frame
			  m_FTEX_handle.rx_frame.ByteCnt = 0;		
        osThreadFlagsSet(TSK_eUART0_handle, EUART_FLAG); // Notify client a frame has been received									
			}
      else
			{
			  eUART_Receive(&m_FTEX_handle.euart_handler, m_FTEX_handle.euart_handler.rx_byte);
			}									
						     
      break;						
	 }			
}

/**@brief Function for sending a frame specific to the FTEX protocol
 * 
 * @param[in] m_FTEX_handle.tx_frame: Frame to send
 */
void LCD_FTEX_TX_IRQ_Handler(void)
{
    uint8_t tx_data;

    if(m_FTEX_handle.tx_frame.ByteCnt < m_FTEX_handle.tx_frame.Size) //Have we sent every byte in the frame
    {
			if(m_FTEX_handle.tx_frame.ByteCnt == 0) //The first byte to send is the code
			{
			  tx_data = m_FTEX_handle.tx_frame.Code;
			}
			else
			{
			  tx_data = m_FTEX_handle.tx_frame.Buffer[m_FTEX_handle.tx_frame.ByteCnt - 1];
			}
			
			m_FTEX_handle.tx_frame.ByteCnt++;
			// Send the data byte 
			eUART_Send(&m_FTEX_handle.euart_handler,&tx_data, 1); 
    }
    else
    {	
			//Resume reception of frames			
			m_FTEX_handle.tx_frame.ByteCnt = 0;
			eUART_Receive(&m_FTEX_handle.euart_handler,m_FTEX_handle.euart_handler.rx_byte); //Restart reception
	  }
}

/**@brief Function for decoding a received frame (previously built on the callback function)
 * 
 * @param[in] rx_frame: Frame that needs to be decoded. 
 */
void LCD_FTEX_frame_Process(void)
{
	FTEX_frame_t replyFrame = {0};
	uint32_t toSend;
	uint16_t CheckSum = 0;
	
	//Check checksum
	
	if(LCD_FTEX_CRC16(m_FTEX_handle.rx_frame) == 0)
	{
	  switch(m_FTEX_handle.rx_frame.Code)
	  {
	  	case FTEX_FRAME_READ: //To be fixed after negociations with eggrider are finished
         
		    if(m_FTEX_handle.rx_frame.Buffer[0] == FTEX_FRAME_READ)
				{
				  replyFrame.Code = FTEX_FRAME_READ;
					replyFrame.Buffer [0] = FTEX_FRAME_READ;
					
				//	toSend = VC_getBattVoltage(m_FTEX_handle.pVController);
					
					toSend = toSend * 100;        //Conversion to 0.01 V / unit from 1 V/ unit
					
					replyFrame.Buffer [1] = 0x13; //toSend & 0xFF; //Write voltage value, every uinit is worth 0.01 V
					replyFrame.Buffer [2] = 0x88; //(toSend >> 8) & 0xFF;
					
				//	toSend = VC_getMotorSpeedMeas(m_FTEX_handle.pVController,M1);
			  
			    toSend = (toSend * 10) / 37;
					
				  replyFrame.Buffer [3] = 0x04; //toSend & 0x00FF; //Write speed value, every uinit is worth 0.01 Km/h
					replyFrame.Buffer [4] = 0xE2; //(toSend >> 8) & 0x00FF;
					
					replyFrame.Buffer [5] = 0x13; //Write Current value, every uinit is worth 0.01 A (N/A)
					replyFrame.Buffer [6] = 0x88;
					
					replyFrame.Buffer [7] = 0x63; //Write Battery % (0-100) (N/A)
					
					replyFrame.Buffer [8] = 0xFF; //Write pedal cadence     (N/A)
					
					replyFrame.Buffer [9] = 0xFF; //Write Human torque      (N/A)
					
					//toSend = VC_getMotorTemp(m_FTEX_handle.pVController,M1);
				
  				toSend += 50;
					
					replyFrame.Buffer[10] = 0xFF; //toSend; //Write motor temp with 0 = -50 celcius
					
					//toSend = VC_getInverterHeatsinkTemp(m_FTEX_handle.pVController,M1);
					
					toSend += 50;
					
					replyFrame.Buffer[11] = 0x32; //toSend; //Write controller temp with 0 = -50 celcius
					
					replyFrame.Buffer[12] = 0x5A; //Write Battery temp with 0 = -50 celcius  (N/A)
					
					replyFrame.Buffer[13] = 0x00; //Write error code 
					
				  replyFrame.Buffer[14] = 0xFF; //Write misc flags (front light, rear light ,etc) (N/A)

          replyFrame.Buffer[15] = 0xFF; //Spare byte
          
					replyFrame.Buffer[16] = 0xFF; //Spare byte
					
					replyFrame.Size = 18; //Set the frame size for the CRC calculation
					
					//Calculate CRC16 checksum
					CheckSum = LCD_FTEX_CRC16(replyFrame);

					
					replyFrame.Buffer[17] = (CheckSum >> 8)& 0x00FF; //Write checksum
					replyFrame.Buffer[18] = (CheckSum) & 0x00FF;
				  
					
					replyFrame.Size = replyFrame.Size + 2; //Add the two CRC bytes to the frame size
				}					
	  	break; //end case FTEX_CMD_READ
		
	  	case FTEX_FRAME_WRITE:
  	
  		break;//end case FTEX_CMD_WRITE
		
		  default: 
	  		replyFrame.Size = 0;
	  	  eUART_Receive(&m_FTEX_handle.euart_handler, m_FTEX_handle.euart_handler.rx_byte);
	  		break;
  	}
 
	}//checksum check
	
	if(replyFrame.Size > 0) //Check if we need to send a response
	{
		replyFrame.ByteCnt = 0;
		
		m_FTEX_handle.tx_frame = replyFrame; 
     		
		LCD_FTEX_TX_IRQ_Handler();
	}	
	else 
	{
		eUART_Receive(&m_FTEX_handle.euart_handler, m_FTEX_handle.euart_handler.rx_byte);
	}
}

/**@brief Function for initializing the LCD module with the EggRider protocol
 * 
 * @param[in] pHandle: Handle for vehicle controller 
 */
void LCD_FTEX_init(VCI_Handle_t * pHandle)
{
		m_FTEX_handle.pVController = pHandle;  					 								    // Pointer to VController
		m_FTEX_handle.euart_handler.dev_type = EUART_FTEX;							    // Attribution of LCD type
	  m_FTEX_handle.euart_handler.p_evt_handler = LCD_FTEX_event_handler; // Attribution of callback function
		m_FTEX_handle.euart_handler.p_uart_inst = UART0_INSTANCE_ADDR;			// Attribution of UART0 adresse
	
		nrf_drv_uart_config_t uart_drive_config =
	  {                                                     
      .pseltxd            = UART0_TX_PIN,               
      .pselrxd            = UART0_RX_PIN,              	
      .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
      .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
      .p_context          = &m_FTEX_handle.euart_handler, // To be able to recover on the uart event handler on the low level  	                  
      .hwfc               = NRF_UART_HWFC_DISABLED,       
      .parity             = NRF_UART_PARITY_EXCLUDED,     
      .baudrate           = NRF_UART_BAUDRATE_9600, 		
      .interrupt_priority = 2,                  		      //TODO lower interrupt priority	for all screens			
      NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	  };
	
	eUART_Init(&m_FTEX_handle.euart_handler,uart_drive_config);
	eUART_Receive(&m_FTEX_handle.euart_handler, m_FTEX_handle.euart_handler.rx_byte);	
}

/**@brief Checksum calculation function
 *
 * @param[in] frame used to calculate the checksum
 *
 * Used to make the 16 bits chesum when sending a frame or
 * to verify a received frame to see if it is valid.
 */
uint16_t LCD_FTEX_CRC16(FTEX_frame_t frame)
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
		 else //the buffer contains the rest
		 {
		   data = frame.Buffer[i-1];
		 }
				 
     x = ((crc >> 8) ^ data) & 0xFF;
     x ^= (x >> 4);
     crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;				
   }		
  return(crc);
}
