/**
  ******************************************************************************
  * @file    lcd_bafang_comm.c
  * @author  Andy Beaudoin, FTEX
	* @author  Jorge Polo, FTEX
  * @brief   High level module that describes Bafang LCD communication protocol
  *
	******************************************************************************
	*/
	
#include "lcd_bafang_comm.h"

// Private handler
BAF_Handle_t m_Baf_handle;
extern osThreadId_t TSK_eUART0_handle; // Task Id for external uart (UART0 instance)

/**@brief Callback used for managing bytes received or sent from the low level layer (euart_manager)
 *
 * @param[in] p_lcd_event: Structure that contains the received byte (or byte to send) and the type of event
 */
static void LCD_Baf_event_handler(eUART_evt_t * p_lcd_event)
{
	switch(p_lcd_event->evt_type)
	{
		case EUART_BYTE_RECEIVED:
		  LCD_BAF_RX_IRQ_Handler(p_lcd_event->byte_to_send[0]);
		  break;
		
		case EUART_BYTE_SENT:
			LCD_BAF_TX_IRQ_Handler();
			break;
	}
}

/**@brief Function for building a frame specific this the bafang protocol
 * 
 * @param[in] rx_frame: Frame that needs to be decoded. 
 */
void LCD_BAF_RX_IRQ_Handler(unsigned short rx_data)
{
	
	if(m_Baf_handle.TrashCnt >= 123) //Skip the bundle of 123 trash bytes when the screen is powerd on
	{
        uint8_t ByteCount    = m_Baf_handle.rx_frame.ByteCnt;     
        uint8_t ByteReceived = rx_data;
				 
        switch(ByteCount)					 
        {
		  case 0:
				 if(ByteReceived == BAF_CMD_READ || ByteReceived == BAF_CMD_WRITE) //Read or write cmd
				 {
					 m_Baf_handle.rx_frame.ByteCnt ++;
					 m_Baf_handle.rx_frame.Code = ByteReceived;
				 }
				 else //If not its a bad cmd
				 {
					 m_Baf_handle.rx_frame.ByteCnt = 0;
				 }
				 // Ask for another byte
				 eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
			 break;
		  
			case 1:
			case 2:
			case 3:
				 m_Baf_handle.rx_frame.ByteCnt ++;
				 m_Baf_handle.rx_frame.Buffer[ByteCount-1] = ByteReceived;	 
				
				 if(m_Baf_handle.rx_frame.Code == BAF_CMD_READ) //Read cmd
				 {	
						m_Baf_handle.rx_frame.ByteCnt = 0;
						osThreadFlagsSet(TSK_eUART0_handle, EUART_FLAG); // Notify task a frame has been received
				 }
				 else if(m_Baf_handle.rx_frame.Code == BAF_CMD_WRITE)//Write cmd
				 {
						if(m_Baf_handle.rx_frame.Buffer[0] == W_ASSIST) 
						{
							if(ByteCount == 3) //Assit cmd needs 4 bytes (0-3)
							{								
								m_Baf_handle.rx_frame.ByteCnt = 0;
								osThreadFlagsSet(TSK_eUART0_handle, EUART_FLAG); // Notify task a frame has been received
							}
						}
						else //Wrong frame, trash it.
						{
							m_Baf_handle.rx_frame.ByteCnt = 0;
							// Ask for another byte
							eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
						}
					}	
				 else //Wrong frame, trash it.
				 {
						m_Baf_handle.rx_frame.ByteCnt = 0;
						// Ask for another byte
						eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
				 }
		   break;		 
		 default:
			 //Unknown frame, trash it.
			 m_Baf_handle.rx_frame.ByteCnt = 0;
			 // Ask for another byte
			 eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
			 break;						
        }
    }
    else
    {
        m_Baf_handle.TrashCnt ++;
        eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
    }	
}

/**@brief Function for sending a response byte by byte
 * 
 * @param[in] the data should be place in m_baf_handle.tx_frame
 *            before calling this function
 */
void LCD_BAF_TX_IRQ_Handler(void)
{
    uint8_t tx_data;

    if(m_Baf_handle.tx_frame.ByteCnt < m_Baf_handle.tx_frame.Size) //Check if the entire frame has been sent
    {
			tx_data = m_Baf_handle.tx_frame.Buffer[m_Baf_handle.tx_frame.ByteCnt];
			m_Baf_handle.tx_frame.ByteCnt ++;
			// Send the data byte 
			eUART_Send(&m_Baf_handle.euart_handler,&tx_data, 1); 
    }
    else
    {	
			//Resume reception of frames			
			m_Baf_handle.tx_frame.ByteCnt = 0;
			eUART_Receive(&m_Baf_handle.euart_handler,m_Baf_handle.euart_handler.rx_byte); //Restart reception
	  }
}

/**@brief Function for decoding a received frame (previously built on the callback function)
 *
 * @param[in] the data should be place in m_baf_handle.rx_frame
 *            before calling this function
 *
 *  Answers to the screen when needed, applies the changes to the controller that are relevant. 
 */
void LCD_BAF_frame_Process(void)
{
	BAF_frame_t replyFrame = {0};
	int32_t toSend    = 0;

	uint8_t AssistLvl = 0;
	
	switch(m_Baf_handle.rx_frame.Code)
	{
		case BAF_CMD_READ:
		{
			switch(m_Baf_handle.rx_frame.Buffer[0])
			{
				case R_VERSION:          //Answer is fixed
				    replyFrame.Size = 3;
				    replyFrame.Buffer[0] = 0x90;
				    replyFrame.Buffer[1] = 0x40;
				    replyFrame.Buffer[2] = 0x0D;
				  break;
				 
			  case R_STATUS:           //0x01 means there are no errors
					  replyFrame.Size = 2; //To change if we want to send errors to the screen
					  replyFrame.Buffer[0] = 0x01;
					  replyFrame.Buffer[1] = 0x01;							   
					break;				
			 
			  case R_WORKSTATUS:       //0x31 -> controller is working
				    replyFrame.Size = 2; 
				    replyFrame.Buffer[0] = 0x31;
				    replyFrame.Buffer[1] = 0x31;
					break;
			 
			  case R_CURRENT:
				    //answer with 0-250 value twice (each unit = 0.5 A)
				
				    //Aprox calculation 
		 		    toSend = abs(MDI_getIq(m_Baf_handle.pVController->pDrivetrain->pMDI,M1)) + 
				             abs(MDI_getIq(m_Baf_handle.pVController->pDrivetrain->pMDI,M2));
					  
				    toSend = (uint8_t)(toSend >> 8)*4/5;
				
			      replyFrame.Size = 2;
				    replyFrame.Buffer[0] = toSend;
				    replyFrame.Buffer[1] = toSend;
					break;
			 
			  case R_BATCAP:
				    //  toSend = VC_getBattVoltage(m_Baf_handle.pVController);
			     
    				toSend = ((toSend - 30) * 5)-1; //Aprox calculation of bat %
			    
			      if(toSend > 100)
				  	{
					    toSend = 99;
					  }
					
				    replyFrame.Size = 2;
				    replyFrame.Buffer[0] = toSend;
				    replyFrame.Buffer[1] = toSend;
					break;
			 
			  case R_RSPEED:
			     
    				//If there is a motor at the rear use it for its speed
				
	          if(MDI_getSpeed(m_Baf_handle.pVController->pDrivetrain->pMDI,M1) > 0)
		        {
		          toSend = MDI_getSpeed(m_Baf_handle.pVController->pDrivetrain->pMDI,M1);		
		        }
		        else //If we are using the other motor use it as the speed reference
		        {
		          toSend = MDI_getSpeed(m_Baf_handle.pVController->pDrivetrain->pMDI,M2); 
		        }	
								
			      //TODO add consideration for the gear ratio between mototr and wheel, for now will assume 1
					 
				   replyFrame.Size = 3;
				   replyFrame.Buffer[0] = (toSend  >> 8) & 0xFF;
				   replyFrame.Buffer[1] = toSend & 0xFF;
				   replyFrame.Buffer[2] = replyFrame.Buffer[0] + replyFrame.Buffer[1] + 0x20; 
				 break;
			 
			  case R_LIGHTSTATUS: //Not supported so we answer with ligghts closed
				   replyFrame.Size = 2;
				   replyFrame.Buffer[0] = 0x00;
				   replyFrame.Buffer[1] = 0x00;
			   break;
			 
			  case R_PHOTTHRESH: //Not supported so we answer with a regular value
				   replyFrame.Size = 2;
				   replyFrame.Buffer[0] = 0xFF;
				   replyFrame.Buffer[1] = 0xFF;
				 break;
			 
			  case SKIP:
			  default:
				   replyFrame.Size = 0;
				   eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
				 break;
			}
		}break; //end case BAF_CMD_READ
		
		case BAF_CMD_WRITE:
		{
			switch(m_Baf_handle.rx_frame.Buffer[0])
			{
				case W_ASSIST:
					 AssistLvl = m_Baf_handle.rx_frame.Buffer[1];
				
				   switch(AssistLvl) //Set the pas level according to what the user selected
					 {
							case A_0:
							  	DRVT_SetPASLevel(m_Baf_handle.pVController->pDrivetrain,PAS_LEVEL_0);	//Set pass to 0							
								break;
							case A_1: //A_1, A_3, A_5, A_7, arent used until we support 9 pass levels
								break;
							case A_2:
						  		DRVT_SetPASLevel(m_Baf_handle.pVController->pDrivetrain,PAS_LEVEL_1); //Set pass to 1		
								break;
							case A_3:
								break;
							case A_4:
						  		DRVT_SetPASLevel(m_Baf_handle.pVController->pDrivetrain,PAS_LEVEL_2); //Set pass to 2	
								break;
							case A_5:
								break;
							case A_6:
								  DRVT_SetPASLevel(m_Baf_handle.pVController->pDrivetrain,PAS_LEVEL_3); //Set pass to 3		
								break;
							case A_7:
								break;
							case A_8:
								  DRVT_SetPASLevel(m_Baf_handle.pVController->pDrivetrain,PAS_LEVEL_4); //Set pass to 4		
								break;
							case A_9:
							  	DRVT_SetPASLevel(m_Baf_handle.pVController->pDrivetrain,PAS_LEVEL_5); //Set pass to 5		
								break;
							case A_PUSH:
								break;												
							case A_LSPEED: 
								break;
							default:
							  	DRVT_SetPASLevel(m_Baf_handle.pVController->pDrivetrain,PAS_LEVEL_0); //In case of unexpected value, set pas to 0
								break;
						}				
					break;
				
				default:
					    replyFrame.Size = 0;
					    eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
					break;
			}
		}break;//end case BAF_CMD_WRITE
		
		default: 
			replyFrame.Size = 0;
		  eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
			break;
	}
	
	if(replyFrame.Size > 0) //If size is bigger than one we have to send a response
	{
		replyFrame.ByteCnt = 0;   
		
    m_Baf_handle.tx_frame = replyFrame; 
     		
		LCD_BAF_TX_IRQ_Handler();
	}	
	else //If not just get ready for the next frame
	{
		replyFrame.Size = 0;
		eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
	}
}

/**@brief Function for initializing the LCD module with the Bafang protocol
 * 
 * @param[in] pHandle: Handle for vehicle controller 
 */
void LCD_BAF_init(VCI_Handle_t * pHandle)
{
		m_Baf_handle.pVController = pHandle;  					 								  // Pointer to VController
		m_Baf_handle.euart_handler.dev_type = EUART_BAFANG;							  // Assignation of LCD type
	  m_Baf_handle.euart_handler.p_evt_handler = LCD_Baf_event_handler; // Assignation of callback function
		m_Baf_handle.euart_handler.p_uart_inst = UART0_INSTANCE_ADDR;			// Assignation of UART0 adresse
	  m_Baf_handle.TrashCnt = 0;
	
		nrf_drv_uart_config_t uart_drive_config =
	  {                                                     
      .pseltxd            = UART0_TX_PIN,               
      .pselrxd            = UART0_RX_PIN,              	
      .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
      .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
      .p_context          = &m_Baf_handle.euart_handler, // To be able to recover on the uart event handler on the low level  	                  
      .hwfc               = NRF_UART_HWFC_DISABLED,       
      .parity             = NRF_UART_PARITY_EXCLUDED,     
      .baudrate           = NRF_UART_BAUDRATE_1200, 		
      .interrupt_priority = 5,                  				
      NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	  };
	
	eUART_Init(&m_Baf_handle.euart_handler,uart_drive_config);
	eUART_Receive(&m_Baf_handle.euart_handler, m_Baf_handle.euart_handler.rx_byte);
}
