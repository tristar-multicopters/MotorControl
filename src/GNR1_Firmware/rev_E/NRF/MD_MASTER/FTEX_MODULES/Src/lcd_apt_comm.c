/**
  ******************************************************************************
  * @file    lcd_apt_comm.c
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes APT LCD communication protocol
  *
	******************************************************************************
	*/
	
#include "lcd_apt_comm.h"

#define APTMAXCURRENT 60
#define LOWEST_GEAR_RATIO 0x00010001
#define HIGEST_GEAR_RATIO 0x00F000F0

// Private handler
APT_Handle_t m_APT_handle;
extern osThreadId_t TSK_eUART0_handle; // Task Id for external uart (UART0 instance)

/**@brief Callback used for managing bytes received or sent from the low level layer (euart_manager)
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
	}
}

/**@brief Function for building a frame specific to the APT protocol
 * 
 * @param[in] rx_frame: Frame that needs to be decoded. 
 */
void LCD_APT_RX_IRQ_Handler(unsigned short rx_data)
{		
	 uint8_t ByteCount = m_APT_handle.rx_frame.ByteCnt;     
   uint8_t ByteReceived = rx_data;
				 
	 switch(ByteCount)					 
	 {
		 case 0:
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
		 case 8: //Every frame has a lenght of 9 bytes (0-8)
							
				if(ByteReceived == APT_END) //Read or write cmd
				 {
					 m_APT_handle.rx_frame.Buffer[ByteCount] = ByteReceived;
					 osThreadFlagsSet(TSK_eUART0_handle, EUART_FLAG); // Notify client a frame has been received
				 }
				 else //If the last byte isnt the end byte, trash the frame
				 {
					 eUART_Receive(&m_APT_handle.euart_handler, m_APT_handle.euart_handler.rx_byte);
				 }
				 m_APT_handle.rx_frame.ByteCnt = 0;
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
 * @param[in] the data should be place in m_APT_handle.tx_fram
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
	 uint32_t GearRatio = 0;
	 uint16_t Merge     = 0;
	 uint8_t  PassLvl   = 0;

	
	 //Verification of the checksum
	 for(int i = 0; i < 6; i += 2) //Checksum is the sum of double bytes into a 16 bits
	 {                             //Single bytes need to be paired into a single 16 bits before being summed    
	 	 Merge = (m_APT_handle.rx_frame.Buffer[i] << 8) + m_APT_handle.rx_frame.Buffer[i+1];
	   Check += Merge;
	 }
	 
	 Check = (Check & 0x0000FFFF); //Protection in case of overflow
	
	 //Check if the CRC is good
   if(Check == m_APT_handle.rx_frame.Buffer[CHECK + 1] + (m_APT_handle.rx_frame.Buffer[CHECK] << 8))
	 {
		   //Reading the Pass
		   PassLvl = (m_APT_handle.rx_frame.Buffer[PASS] & 0x0F); //Only the 4 LSB contain the pass level
		
       if(PassLvl < 0xA) //We currently only support 5 levels of pass
		   {			
			   switch(PassLvl)
			    {
				   case 0:
					    DRVT_SetPASLevel(m_APT_handle.pVController->pDrivetrain,PAS_LEVEL_0); //Set pass to 0
            break;							
				   case 1:
					    DRVT_SetPASLevel(m_APT_handle.pVController->pDrivetrain,PAS_LEVEL_1); //Set pass to 1
            break;	
				   case 3:
					    DRVT_SetPASLevel(m_APT_handle.pVController->pDrivetrain,PAS_LEVEL_2); //Set pass to 2
            break;	
				   case 5:
					    DRVT_SetPASLevel(m_APT_handle.pVController->pDrivetrain,PAS_LEVEL_3); //Set pass to 3
            break;	
				   case 7:
					    DRVT_SetPASLevel(m_APT_handle.pVController->pDrivetrain,PAS_LEVEL_4); //Set pass to 4
            break;								
				   case 9:
					    DRVT_SetPASLevel(m_APT_handle.pVController->pDrivetrain,PAS_LEVEL_5); //Set pass to 5
            break;				
		  	  }
 		   }			
		
	     //Reading the Speed   limit  TBA
	  	 //Reading the Current limit  TBA		
		   //Reading the Wheel diameter TBA	   
	 
	
	    //For APT protocol, LSB is sent first for multi-bytes values
		  replyFrame.Size = 13;
	
      replyFrame.Buffer[ 0] = APT_START; //Start
	  
	  	//Add up power from both
	  	toSend = abs(MDI_getIq(m_APT_handle.pVController->pDrivetrain->pMDI,M1)) + 
	             abs(MDI_getIq(m_APT_handle.pVController->pDrivetrain->pMDI,M2));
		
				
		  toSend = toSend/(0x7FFF/APTMAXCURRENT); //Conversion from relative current to actual amps
      toSend = toSend * 2;                    //Covert from amps to 0.1 amps; 
		
      replyFrame.Buffer[ 1] = (toSend & 0x000000FF); //Power 0.1 A/unit 		


		  toSend = abs(MDI_getSpeed(m_APT_handle.pVController->pDrivetrain->pMDI,M1));		

			
			GearRatio = m_APT_handle.pVController->pDrivetrain->sParameters.GearRatio;  //Gear ratio (motor compared to wheel) is split. 
			                                                                            //msb 16 bits is the numerator, 
			                                                                            //lsb 16 bits is denominator
			                                                                            //ex: 3/2 ratio would be 0x00030002 
			                                                                            //default should be 0x00010001 
			
			if(GearRatio < LOWEST_GEAR_RATIO || GearRatio > HIGEST_GEAR_RATIO) //valid values of gear ratio
			{
			   GearRatio = LOWEST_GEAR_RATIO;
		  }				
			
			toSend = ((GearRatio & 0xFFFF0000) >> 16) * toSend / (GearRatio & 0x0000FFFF);
		
		
		  toSend = toSend * 500;       //Converion from RPM to period in ms 
		                               //
	    toSend = 500000/(toSend/60); //
	
	    replyFrame.Buffer[ 2] = (toSend & 0x00FF);      //Motor speed Low half 
	    replyFrame.Buffer[ 3] = (toSend & 0xFF00) >> 8; //Motor speed High half
	  
	    replyFrame.Buffer[ 4] = 0x00; //Error Code
	  
	    replyFrame.Buffer[ 5] = 0x04; //Brake Code bXXXX 0100 means the motor is working
   
      replyFrame.Buffer[ 6] = 0x00; //Reserved
      replyFrame.Buffer[ 7] = 0x00; //Reserved 
      replyFrame.Buffer[ 8] = 0x00; //Reserved
      replyFrame.Buffer[ 9] = 0x00; //Reserved
	
	    //Calculate checksum
		  //Sum of paired bytes
		  Merge = 0;
		  Check = 0;
	    for(int i = 0; i < 10;	i += 2)
	    {
			  Merge = (replyFrame.Buffer[i+1] << 8) + replyFrame.Buffer[i];
	      Check += Merge;
	    }
		
		  replyFrame.Buffer[10] =  (0x000000FF & Check);       //Checksum Low  Half
	    replyFrame.Buffer[11] = ((0x0000FF00 & Check) >> 8); //Checksum High Half	
	  	  
	    replyFrame.Buffer[12] = APT_END; //End
			
		  replyFrame.ByteCnt = 0;   
		
      m_APT_handle.tx_frame = replyFrame; 
     		
		  LCD_APT_TX_IRQ_Handler(); 
			
   }//End of CRC check
}

/**@brief Function for initializing the LCD module with the APT protocol
 * 
 * @param[in] pHandle: Handle for vehicle controller 
 */
void LCD_APT_init(VCI_Handle_t * pHandle)
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
      .interrupt_priority = 5,                  				 
      NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	  };
	
	eUART_Init(&m_APT_handle.euart_handler,uart_drive_config);
	eUART_Receive(&m_APT_handle.euart_handler, m_APT_handle.euart_handler.rx_byte);
}
