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

/****************** PARAMETERS ***********************/
static LCD_handle_t m_Baf_handle;				// private instance for managing Bafang display communication

osThreadId_t TSK_LCD_Bafcomm_handle;    // Bafang ID task

// Queue declarations
static osMessageQueueId_t Baf_frame_queue;

static const osMessageQueueAttr_t queueAtr_Baf_frame = {
		.name = "Baffang_frame_Q",
};

/***********************************************************/

static void LCD_Baf_event_handler(ufcp_evt_t * p_ufcp_event)
{	
	FCP_Frame_t rx_frame = p_ufcp_event->frame;
	
	switch (p_ufcp_event->evt_type)
	{
			case UFCP_FRAME_RECEIVED:
					osMessageQueuePut(Baf_frame_queue, &rx_frame, NULL, 0);
					osThreadFlagsSet(TSK_LCD_Bafcomm_handle, LCD_BAFFANG_FLAG);
					break;
			
			case UFCP_FRAME_SENT:
					break;
			
			case UFCP_CRC_ERROR:
					break;
			
			default:
					break;
	}
}

static void LCD_BAF_frame_received(FCP_Frame_t * rx_frame)
{
	FCP_Frame_t replyFrame = {0};
	int32_t toSet  = 0;
	int32_t toSend = 0;
	uint8_t AssistLvl = 0;
	
	switch(rx_frame->Code)
	{
		case BAF_CMD_READ:
		{
			switch(rx_frame->Buffer[0])
			{
				case R_VERSION:
					//answer with 0x90 0x40 0x0D
				  replyFrame.Size = 3;
				  replyFrame.Buffer[0] = 0x90;
				  replyFrame.Buffer[1] = 0x40;
				  replyFrame.Buffer[2] = 0x0D;
				  break;
				
			 case R_STATUS:
				 //answer with 0x01 twice
					replyFrame.Size = 2;
					replyFrame.Buffer[0] = 0x01;
					replyFrame.Buffer[1] = 0x01;							   
					break;				
			 
			 case R_WORKSTATUS:
				  //answer with 0x31 0x31
				  replyFrame.Size = 2;
				  replyFrame.Buffer[0] = 0x31;
				  replyFrame.Buffer[1] = 0x31;
					break;
			 
			 case R_CURRENT:
				 // answer with 0-250 value twice 
				 // Jorge: current is a int16_t value. Has to ask to Sami what kind of value we have to send to the screen then...
			 //toSend = VC_getMotorIqIdMeas(m_Baf_handle.pVController,M1).q; //Andy: This is the wrong value, the value that we need doesn't exist
			 
				 // toSend = m_Baf_handle.pVController->pThrottle->hInstThrottle; //Showing the throttle instead could be a replacement
			    toSend = 0;
			    replyFrame.Size = 2;
				  replyFrame.Buffer[0] = (toSend >> 8) & 0xFF;
				  replyFrame.Buffer[1] = toSend & 0xFF;
					break;
			 
			 case R_BATCAP:
				 //answer with 0-99 value twice
				  toSend = VC_getBattVoltage(m_Baf_handle.pVController);
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
				 //answer with rpm + checksum (speed + 0x20)
			   //Jorge: speed is coded in 4 bytes, mesure is in RPM.
				 toSend = VC_getMotorSpeedMeas(m_Baf_handle.pVController,M1);
			 
			   toSend = (toSend * 10) / 37; 
				 replyFrame.Size = 3;
				 replyFrame.Buffer[0] = (toSend  >> 8) & 0xFF;
				 replyFrame.Buffer[1] = toSend & 0xFF;
				 replyFrame.Buffer[2] = replyFrame.Buffer[0] + replyFrame.Buffer[1] + 0x20; 
				 break;
			 
			 case R_LIGHTSTATUS:
				 //answer with 0x00 0x00 (light off)
				 replyFrame.Size = 2;
				 replyFrame.Buffer[0] = 0x00;
				 replyFrame.Buffer[1] = 0x00;
			   break;
			 
			 case R_PHOTTHRESH:
				 //answer with threshhold 0xFF 0xFF
				 replyFrame.Size = 2;
				 replyFrame.Buffer[0] = 0xFF;
				 replyFrame.Buffer[1] = 0xFF;
				 break;
			 
			 case SKIP:
			 default:
				 replyFrame.Size = 0;
				 UFCP_Receive(&m_Baf_handle.lcd_ufcp_handle);  // TO VERIFY
				break;
			}
		}break; // end case BAF_CMD_READ
		
		case BAF_CMD_WRITE:
		{
			switch(rx_frame->Buffer[1])
			{
				case W_ASSIST:
					 AssistLvl = rx_frame->Buffer[2];
				
				   switch(AssistLvl)
											{
											  case A_0:
													//Speed = 0;   // 0 km/h
                          break;
											  case A_1:
													//Speed = 16;  // 2 km/h
                          break;
											  case A_2:
													//Speed = 31;  // 4 km/h
												  PAS_SetLevel(m_Baf_handle.pVController->pPedalAssist,1);
                          break;
											  case A_3:
													//Speed = 47;  // 6 km/h
                          break;
											  case A_4:
													//Speed = 62;  // 8 km/h
												  PAS_SetLevel(m_Baf_handle.pVController->pPedalAssist,2);
                          break;
											  case A_5:
													//Speed = 78;  //10 km/h
                          break;
											  case A_6:
													//Speed = 93;  //12 km/h
												  PAS_SetLevel(m_Baf_handle.pVController->pPedalAssist,3);
                          break;
											  case A_7:
													//Speed = 109; //14 km/h
                          break;
											  case A_8:
													//Speed = 124; //16 km/h
												  PAS_SetLevel(m_Baf_handle.pVController->pPedalAssist,4);
                          break;
											  case A_9:
													//Speed = 140; //18 km/h
											  	PAS_SetLevel(m_Baf_handle.pVController->pPedalAssist,5);
                          break;
											  case A_PUSH:
													//Speed = 202; //26 km/h
                          break;												
											  case A_LSPEED: // 1 km/h
													//Speed = 8;
                          break;
												default:
													PAS_SetLevel(m_Baf_handle.pVController->pPedalAssist,0);
													//Speed = 247; //32 km/h
													break;
											}				
					break;
				
				default:
					    UFCP_Receive(&m_Baf_handle.lcd_ufcp_handle);
					break;
			}
		}break; // end case BAF_CMD_WRITE
		
		default: // TODO: Rise an error
			UFCP_Receive(&m_Baf_handle.lcd_ufcp_handle);
			break;
	}
	
	if(replyFrame.Size > 0)
	{
		UFCP_Send(&m_Baf_handle.lcd_ufcp_handle,replyFrame.Code,replyFrame.Buffer,replyFrame.Size);
	}
	
	else // TO VERIFY...
	{
		UFCP_Receive(&m_Baf_handle.lcd_ufcp_handle);
	}
}

void LCD_Baf_Init(VC_Handle_t * pHandle)
{
		m_Baf_handle.pVController = pHandle;  // Pointer to VController
		nrf_drv_uart_config_t uart_drive_config =
	{                                                     
    .pseltxd            = UART0_TX_PIN,               
    .pselrxd            = UART0_RX_PIN,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = &m_Baf_handle,                         
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_1200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	m_Baf_handle.lcd_ufcp_handle.p_uart_inst = UART0_INSTANCE_ADDR;
	
	UFCP_Init(&m_Baf_handle.lcd_ufcp_handle,uart_drive_config,LCD_Baf_event_handler);
}

__NO_RETURN void TSK_LCD_Baf_comm (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	Baf_frame_queue = osMessageQueueNew(LCD_PENDING_BUFFER_SIZE, sizeof(FCP_Frame_t),&queueAtr_Baf_frame);
	FCP_Frame_t rx_frame;
	
	UFCP_Receive(&m_Baf_handle.lcd_ufcp_handle);
	while (true)
	{
		osThreadFlagsWait(LCD_BAFFANG_FLAG, osFlagsWaitAny, osWaitForever);
		
		if ( osMessageQueueGet(Baf_frame_queue, &rx_frame, NULL, 0) == osOK )
		{
			LCD_BAF_frame_received(&rx_frame);
		}
	}
}
