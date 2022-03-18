/**
  ******************************************************************************
  * @file    lcd_apt_comm.c
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes APT LCD communication protocol
  *
	******************************************************************************
	*/
	
#include "lcd_apt_comm.h"

// Private handler
APT_Handle_t m_APT_handle;
extern osThreadId_t TSK_eUART0_handle; // Task Id for external uart (UART0 instance)

/**@brief Callback used for managing bytes received or sent from the low level layer (lcd_comm_manager)
 *
 * @param[in] p_lcd_event: Structure that contains the received byte (or byte to send) and the type of event
 */
static void LCD_APT_event_handler(eUART_evt_t * p_lcd_event)
{
	switch(p_lcd_event->evt_type)
	{
		case EUART_BYTE_RECEIVED:
		  LCD_APT_RX_IRQ_Handler(p_lcd_event->byte_to_send[0]);
		  break;
		
		case EUART_BYTE_SENT:
			LCD_APT_TX_IRQ_Handler();
			break;
		
		default:
			while(1);
			break;
	}
}

/**@brief Function for building a frame specific this the bafang protocol
 * 
 * @param[in] rx_frame: Frame that needs to be decoded. 
 */

void * LCD_APT_RX_IRQ_Handler(unsigned short rx_data)
{		
	 uint8_t ByteCount = m_APT_handle.rx_frame.ByteCnt;     
   uint8_t ByteReceived = rx_data;
				 
	 switch(ByteCount)					 
	 {
		 case 0:
			 {
				 if(ByteReceived == APT_START) //Read or write cmd
				 {
					 m_APT_handle.rx_frame.Buffer[ByteCount] = ByteReceived;
					 m_APT_handle.rx_frame.ByteCnt ++;
					 
				 }
				 else //If not, its a bad cmd
				 {
					 m_APT_handle.rx_frame.ByteCnt = 0;
				 }
				 // Ask for another byte
				 eUART_Receive(&m_APT_handle.euart_handler, m_APT_handle.euart_handler.rx_byte);
			 }break;
		 
			case 8:
				
			  
			
				if(ByteReceived == APT_END) //Read or write cmd
				 {
					 m_APT_handle.rx_frame.Buffer[ByteCount] = ByteReceived;
					 osThreadFlagsSet(TSK_eUART0_handle, EUART_FLAG); // Notify client a frame has been received
				 }
				 else 
				 {
					 eUART_Receive(&m_APT_handle.euart_handler, m_APT_handle.euart_handler.rx_byte);
				 }
				 m_APT_handle.rx_frame.ByteCnt = 0;
			break;
		  case 1:
			case 2:
			case 3:
      case 4:
      case 5:
      case 6:
      case 7:	
				 m_APT_handle.rx_frame.Buffer[ByteCount] = ByteReceived;
				 m_APT_handle.rx_frame.ByteCnt ++;
			
			   eUART_Receive(&m_APT_handle.euart_handler, m_APT_handle.euart_handler.rx_byte);
		  break;
		 
		 default:
			   m_APT_handle.rx_frame.ByteCnt = 0;
			   // Ask for another byte
			   eUART_Receive(&m_APT_handle.euart_handler, m_APT_handle.euart_handler.rx_byte);
			 break;						
	 }
  	
}

/**@brief Function for sending a response byte by byte
 * 
 * @param[in] the data should be place in m_baf_handle.tx_fram
 *            before calling this function
 */
void LCD_APT_TX_IRQ_Handler(void)
{
    uint8_t tx_data;

    if(m_APT_handle.tx_frame.ByteCnt < m_APT_handle.tx_frame.Size)
    {
			tx_data = m_APT_handle.tx_frame.Buffer[m_APT_handle.tx_frame.ByteCnt];
			m_APT_handle.tx_frame.ByteCnt ++;
			// Send the data byte 
			eUART_Send(&m_APT_handle.euart_handler,&tx_data, 1); 
    }
    else
    {	
			//Resume reception of frames			
			m_APT_handle.tx_frame.ByteCnt = 0;
			eUART_Receive(&m_APT_handle.euart_handler,m_APT_handle.euart_handler.rx_byte); //Restart reception
	  }

}

/**@brief Function for decoding a received frame (previously built on the callback function)
 * 
 * @param[in] rx_frame: Frame that needs to be decoded. 
 */

void LCD_APT_frame_Process(void)
{
	APT_frame_t replyFrame = {0};
	int32_t  toSend    = 0;
	uint32_t Check     = 0;
	uint16_t Merge     = 0;
	uint8_t  PassLvl   = 0;
	uint8_t  WheelSize = 0;
	uint8_t  Sensor    = 0;
	
	//Verification of the checksum
	for(int i = 0; i < 6; i += 2) //Checksum is the sum of double bytes into a 16 bits
	{                             //Single bytes need to be paired into a single 16 bits before being summed    
		Merge = (m_APT_handle.rx_frame.Buffer[i] << 8) + m_APT_handle.rx_frame.Buffer[i+1];
	  Check += Merge;
	}
	 
	Check = (Check & 0x0000FFFF);
	
  if(Check == m_APT_handle.rx_frame.Buffer[CHECK + 1] + (m_APT_handle.rx_frame.Buffer[CHECK] << 8))
	{
		//Reading the Pass
		PassLvl = (m_APT_handle.rx_frame.Buffer[PASS] & 0x0F); // Only the 4 LSB contain the pass level
		
    if(PassLvl < 0xA)
		{
			
			switch(PassLvl)
			{
				case 0:
					PAS_SetLevel(m_APT_handle.pVController->pPedalAssist,PAS_LEVEL_0);
       break;							
				case 1:
					PAS_SetLevel(m_APT_handle.pVController->pPedalAssist,PAS_LEVEL_1);
       break;	
				case 2:
					PAS_SetLevel(m_APT_handle.pVController->pPedalAssist,PAS_LEVEL_2);
       break;	
				case 3:
					PAS_SetLevel(m_APT_handle.pVController->pPedalAssist,PAS_LEVEL_3);
       break;	
				case 4:
					PAS_SetLevel(m_APT_handle.pVController->pPedalAssist,PAS_LEVEL_4);
       break;								
				case 5:
					PAS_SetLevel(m_APT_handle.pVController->pPedalAssist,PAS_LEVEL_5);
       break;	
				default:
					while(1); //Pass level not supported ? check screen settings
       break;					
			}
  	 	
		}			

		
	  //Reading the Speed   limit TBA
		//Reading the Current limit TBA
		
		//Reading the Wheel diameter
	  WheelSize = m_APT_handle.rx_frame.Buffer[WHEELD];
		
		if(WheelSize < 35) //Wheel diameters size is in inches
		{
			 
		}
		else if(WheelSize > 50) //Wheel perimiter size is in centimeters
		{
		 
		}
		else //Wheel size is unexpect
		{
		 while(1);
		} 	   
	}
  else
	{
	  while(1); //Checksum isnt valid		
	}	
	
	  // For APT protocol, LSB is sent first for multi-byte values
		replyFrame.Size = 13;
	
    replyFrame.Buffer[ 0] = APT_START; //Start
	
	
	 // toSend = m_APT_handle.pVController->pThrottle->hInstThrottle; //Showing the throttle instead could be a replacement
	
	 /* if(toSend > 0xFF)
		{
		  toSend = 0xFF;
		}*/	
		toSend = 0x64;
    replyFrame.Buffer[ 1] = (toSend & 0x000000FF); // Power 0.1 A / unit 
	
	
	  toSend = VC_getMotorSpeedMeas(m_APT_handle.pVController,M1);
	
	  toSend = 1000/(toSend/60); //Converion from RPM to period in ms 
	
	  replyFrame.Buffer[ 2] = (toSend & 0x00FF);      // Motor speed Low half 
	  replyFrame.Buffer[ 3] = (toSend & 0xFF00) >> 8; // Motor speed High half
	  
	  replyFrame.Buffer[ 4] = 0x00; // Error Code
	  
	  replyFrame.Buffer[ 5] = 0x04; // Brake Code bXXXX 0100 means the motor is working
   
    replyFrame.Buffer[ 6] = 0x00; // Reserved
    replyFrame.Buffer[ 7] = 0x00; // Reserved 
    replyFrame.Buffer[ 8] = 0x00; // Reserved
    replyFrame.Buffer[ 9] = 0x00; // Reserved
	
	  //Calculate checksum
		Merge = 0;
		Check = 0;
	  for(int i = 0; i < 10;	i += 2)
	  {
			Merge = (replyFrame.Buffer[i+1] << 8) + replyFrame.Buffer[i];
	    Check += Merge;
	  }
		
		replyFrame.Buffer[10] =  (0x000000FF & Check);       // Checksum Low  Half
	  replyFrame.Buffer[11] = ((0x0000FF00 & Check) >> 8); // Checksum High Half	
	  	  
	  replyFrame.Buffer[12] = APT_END; //End
			
		replyFrame.ByteCnt = 0;   
		
    m_APT_handle.tx_frame = replyFrame; 
     		
		LCD_APT_TX_IRQ_Handler();
}

/**@brief Function for initializing the LCD module with the Bafang protocol
 * 
 * @param[in] pHandle: Handle for vehicle controller 
 */
void LCD_APT_init(VC_Handle_t * pHandle)
{
		m_APT_handle.pVController = pHandle;  					 								  // Pointer to VController
		m_APT_handle.euart_handler.dev_type = EUART_APT;							    // Assignation of LCD type
	  m_APT_handle.euart_handler.p_evt_handler = LCD_APT_event_handler; // Assignation of callback function
		m_APT_handle.euart_handler.p_uart_inst = UART0_INSTANCE_ADDR;			// Assignation of UART0 adresse
	
		nrf_drv_uart_config_t uart_drive_config =
	  {                                                     
      .pseltxd            = UART0_TX_PIN,               
      .pselrxd            = UART0_RX_PIN,              	
      .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
      .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
      .p_context          = &m_APT_handle.euart_handler, // To be able to recover on the uart event handler on the low level  	                  
      .hwfc               = NRF_UART_HWFC_DISABLED,       
      .parity             = NRF_UART_PARITY_EXCLUDED,     
      .baudrate           = NRF_UART_BAUDRATE_9600, 		
      .interrupt_priority = 2,                  					
      NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	  };
	
	eUART_Init(&m_APT_handle.euart_handler,uart_drive_config);
	eUART_Receive(&m_APT_handle.euart_handler, m_APT_handle.euart_handler.rx_byte);
}
