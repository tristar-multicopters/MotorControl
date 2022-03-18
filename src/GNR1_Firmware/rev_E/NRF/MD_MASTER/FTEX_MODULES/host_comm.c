/**
  ******************************************************************************
  * @file    host_comm.c
  * @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
  * @brief   High level module that controls communication between host and nRF52					 
  *
	******************************************************************************
	*/
	
#include "host_comm.h"

extern osThreadId_t TSK_eUART0_handle; // Task Id for external uart (UART0 instance)
extern osThreadId_t TSK_STRG_handle;   // Task for Storage management

HOST_Handle_t m_host_handle;
static void HOST_event_handler(eUART_evt_t * host_evt)
{	
	uint8_t rx_byte = host_evt->byte_to_send[0];
	switch(host_evt->evt_type)
	{
		case EUART_BYTE_RECEIVED:
			HOST_RX_IRQ_Handler(rx_byte);
			break;
		
		case EUART_BYTE_SENT:
			HOST_TX_IRQ_Handler();
			break;
		
		default:
			break;
	}
}

static void HOST_RX_IRQ_Handler( unsigned short rx_data )
{
  FCP_Handle_t * pBaseHandle = & m_host_handle.frame_handle;
  uint8_t error_code;

	uint8_t rx_byte = (uint8_t) rx_data;

	switch ( pBaseHandle->RxFrameLevel )
	{
		case 0: // First Byte received --> The Code
			pBaseHandle->RxFrame.Code = rx_byte;

			/* Start Rx Timeout */
			pBaseHandle->RxTimeoutCountdown = pBaseHandle->RxTimeout;
			pBaseHandle->RxFrameLevel++;
		
			eUART_Receive(&m_host_handle.euart_handler, m_host_handle.euart_handler.rx_byte);
			break;

		case 1: // Second Byte received --> Size of the payload
			pBaseHandle->RxFrame.Size = rx_byte;
			pBaseHandle->RxFrameLevel++;
			if ( pBaseHandle->RxFrame.Size >= FCP_MAX_PAYLOAD_SIZE)
			{ /* Garbage data received decoded with a payload size that exceeds max*/
				pBaseHandle->RxFrameLevel =0 ;
			}
			eUART_Receive(&m_host_handle.euart_handler, m_host_handle.euart_handler.rx_byte);
			break;

		default: // In the payload or the "CRC"
			if ( pBaseHandle->RxFrameLevel < pBaseHandle->RxFrame.Size + FCP_HEADER_SIZE )
			{
				// read byte is for the payload
				pBaseHandle->RxFrame.Buffer[pBaseHandle->RxFrameLevel - FCP_HEADER_SIZE] = rx_byte;
				pBaseHandle->RxFrameLevel++;
				
				eUART_Receive(&m_host_handle.euart_handler, m_host_handle.euart_handler.rx_byte);
			}
			else
			{
				// read byte is for the "CRC"
				pBaseHandle->RxFrame.FrameCRC = rx_byte;

				/* Disable the reception IRQ */
				//nrf_drv_uart_rx_disable(pHandle->p_uart_inst);//LL_USART_DisableIT_RXNE(pHandle->USARTx); *********************************
				/* Indicate the reception is complete. */
				pBaseHandle->RxFrameState = FCP_TRANSFER_IDLE;

				/* Check the Control Sum */
				if ( FCP_CalcCRC( & pBaseHandle->RxFrame ) == pBaseHandle->RxFrame.FrameCRC )
				{
					pBaseHandle->RxFrameLevel = 0; // Reset Rx frame index
					/* OK. the frame is considered correct. Let's forward to client. */
					osThreadFlagsSet(TSK_eUART0_handle, EUART_FLAG);
				}
				else
				{	// Bad CRC				
					m_host_handle.hError = HOST_BAD_CRC;
				}
			}
	} /* end of switch ( pBaseHandle->RxFrameLevel ) */
}

/*
 *
 */
static void HOST_TX_IRQ_Handler()
{
	FCP_Handle_t *p_baseHandle = &m_host_handle.frame_handle;
  if ( FCP_TRANSFER_IDLE != p_baseHandle->TxFrameState )
  {
    uint16_t tx_data;

    switch ( p_baseHandle->TxFrameLevel )
    {
      case 0:
        tx_data = (uint16_t) p_baseHandle->TxFrame.Code;
        break;

      case 1:
        tx_data = (uint16_t) p_baseHandle->TxFrame.Size;
        break;

      default:
        if ( p_baseHandle->TxFrameLevel < p_baseHandle->TxFrame.Size + FCP_HEADER_SIZE )
        {
          tx_data = (uint16_t) p_baseHandle->TxFrame.Buffer[ p_baseHandle->TxFrameLevel - FCP_HEADER_SIZE ];
        }
        else
        {
          tx_data = (uint16_t) p_baseHandle->TxFrame.FrameCRC;
        }
    } /* end of switch ( pBaseHandle->TxFrameLevel ) */

    /* Send the data byte */
		uint8_t tx_data8 = (uint8_t)tx_data;
    eUART_Send(&m_host_handle.euart_handler,&tx_data8, 1); //LL_USART_TransmitData8(pHandle->USARTx, tx_data); *********************************

    if ( p_baseHandle->TxFrameLevel < p_baseHandle->TxFrame.Size + FCP_HEADER_SIZE )
    {
      p_baseHandle->TxFrameLevel++;
    }
    else
    {		// Restart communication
			FCP_Init(p_baseHandle); // reset Tx buffer
			eUART_Receive(&m_host_handle.euart_handler,m_host_handle.euart_handler.rx_byte); //Restart reception
    }
  } /* end of if ( FCP_TRANSFER_IDLE != pBaseHandle->TxFrameState ) */
}

void HOST_frame_received_protocol( void )
{
	int32_t toSend;
	int32_t toSet;
	FCP_Frame_t *rx_frame = &m_host_handle.frame_handle.RxFrame;
	FCP_Frame_t *frame_to_send = &m_host_handle.frame_handle.TxFrame;
	uint8_t bMotorSelection = ( rx_frame->Code & 0xE0) >> 5; // Mask: 1110|0000 
	
	if ( bMotorSelection != 0 ) 	
	{
		bMotorSelection--;  // bMotorSelection = M1 when bMotorSelection = 1
												// bMotorSelection = M2 when bMotorSelection = 0
	}
	
	switch ( rx_frame->Code )
	{
		case VC_PROTOCOL_CODE_GET_FW_VERSION:
		{
			toSend = FW_VERSION;
			frame_to_send->Code = ACK_NOERROR;
			frame_to_send->Size = 1;
			frame_to_send->Buffer[0] = toSend & 0xff;
		} break;
		
		case VC_PROTOCOL_CODE_EXECUTE_CMD:
		{
			switch(rx_frame->Buffer[0])
			{
				case VC_PROTOCOL_CMD_ENTER_CONFIG:
					//TO CODE: Indicate to the systeme we're in config mode
					break;
				
				case VC_PROTOCOL_CMD_OVERWRITE_FLASH:
				/*	//Overwrite nRF flash memory
					STRG_setOverWrite(true);
					osThreadFlagsSet(TSK_STRG_handle, STRG_FLAG);
					while(STRG_isOverWritting()); //To verify*/
					break;
				
				default:
					m_host_handle.hError |= HOST_BAD_REG_ID;
					break;
			}
			frame_to_send->Code = ACK_NOERROR;
			frame_to_send->Size = 0;
			
		}break; // End of case VC_PROTOCOL_CODE_EXECUTE_CMD
			
		case VC_PROTOCOL_CODE_GET_REG:
		{
			switch (rx_frame->Buffer[0])
				{
					case VC_RT_PROTOCOL_REG_V_FLAGS:
						toSend = VC_getVehicleFaultNow(m_host_handle.pVController) << 16 |
										 VC_getVehicleFaultOccurred(m_host_handle.pVController);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 4;
						frame_to_send->Buffer[0] = toSend & 0xff;					// LSB of fault occurred 
						frame_to_send->Buffer[1] = (toSend >> 8)  & 0xff;	// MSB of fault occurred
						frame_to_send->Buffer[2] = (toSend >> 16) & 0xff; // LSB of fault now
						frame_to_send->Buffer[3] = (toSend >> 24) & 0xff;	// MSB of fault now
						break;
					
					case VC_RT_PROTOCOL_REG_V_STATUS:
						toSend = VC_getVehicleState(m_host_handle.pVController);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 1;
						frame_to_send->Buffer[0] = toSend; 
						break;
					
					case VC_RT_PROTOCOL_REG_V_BUS_VOLTAGE:
						toSend = VC_getBattVoltage(m_host_handle.pVController);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff; // LSB
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_V_THROTTLE:
						toSend = VC_getThrottle(m_host_handle.pVController);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff;        // LSB
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_V_BRAKE:
						toSend = VC_isBrakeOn(m_host_handle.pVController);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 1;
						frame_to_send->Buffer[0] = toSend; 
						break;
					
					case VC_RT_PROTOCOL_REG_M_FLAGS:
						toSend = VC_getMotorFaultNow(m_host_handle.pVController,bMotorSelection) << 16 |
										 VC_getMotorFaultOccurred(m_host_handle.pVController,bMotorSelection);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 4;
						frame_to_send->Buffer[0] = toSend & 0xff;					// LSB of fault occurred 
						frame_to_send->Buffer[1] = (toSend >> 8)  & 0xff;	// MSB of fault occurred
						frame_to_send->Buffer[2] = (toSend >> 16) & 0xff; // LSB of fault now
						frame_to_send->Buffer[3] = (toSend >> 24) & 0xff;	// MSB of fault now
						break;
					
					case VC_RT_PROTOCOL_REG_M_STATUS:
						toSend = VC_getMotorState(m_host_handle.pVController, bMotorSelection);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 1;
						frame_to_send->Buffer[0] = toSend;
						break;
					
					case VC_RT_PROTOCOL_REG_M_TORQUE_KP:
					//	toSend = VC_getMotorTorqueKp(m_host_handle.pVController, bMotorSelection);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff;					// LSB of torque Kp
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff;	// MSB of torque kp
						break;
					
					case VC_RT_PROTOCOL_REG_M_TORQUE_KI:
					//	toSend = VC_getMotorTorqueKi(m_host_handle.pVController, bMotorSelection);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff;					// LSB of torque Ki
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff;	// MSB of torque Ki
						break;
					
					case VC_RT_PROTOCOL_REG_M_FLUX_KP:
					//	toSend = VC_getMotorFluxKp(m_host_handle.pVController, bMotorSelection);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff;					// LSB of flux Kp
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff;	// MSB of flux Kp
						break;
					
					case VC_RT_PROTOCOL_REG_M_FLUX_KI:
					//	toSend = VC_getMotorFluxKi(m_host_handle.pVController, bMotorSelection);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff;					// LSB of flux Ki
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff;	// MSB of flux Ki
						break;
					
					case VC_RT_PROTOCOL_REG_M_HEATS_TEMP:
						toSend = VC_getMotorTemp(m_host_handle.pVController, bMotorSelection);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff; 			 // LSB
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_M_TORQUE_MEAS:
						toSend = VC_getMotorIqIdMeas(m_host_handle.pVController, bMotorSelection).q;
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff;        // LSB
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_M_FLUX_MEAS:
						toSend = VC_getMotorIqIdMeas(m_host_handle.pVController, bMotorSelection).d;
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 2;
						frame_to_send->Buffer[0] = toSend & 0xff;        // LSB
						frame_to_send->Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_A:
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_B:
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_Q:
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_D:
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_Q_REF:
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_D_REF:
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_V_Q:
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_V_D:
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_MEAS_ROT_SPEED:
						toSend = VC_getMotorSpeedMeas(m_host_handle.pVController, bMotorSelection);
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 4;
						frame_to_send->Buffer[0] = toSend & 0xff;					// LSB of Speed 
						frame_to_send->Buffer[1] = (toSend >> 8)  & 0xff;	
						frame_to_send->Buffer[2] = (toSend >> 16) & 0xff; 
						frame_to_send->Buffer[3] = (toSend >> 24) & 0xff;	// MSB of Speed
						break;
					
					case VC_CFG_PROTOCOL_REG_V_DUALMOTOR:
				#if IS_DUAL
						toSend = DUAL_M;
				#else
						toSend = SINGLE_M;
				#endif
						frame_to_send->Code = ACK_NOERROR;
						frame_to_send->Size = 1;
						frame_to_send->Buffer[0] = toSend;
						break;
					
					default:
						m_host_handle.hError |= HOST_BAD_REG_ID;
						break;
				}
		}break; // End of case VC_PROTOCOL_CODE_GET_REG
		
		case VC_PROTOCOL_CODE_SET_REG:
		{
			switch(rx_frame->Buffer[0])
			{
				//=========================  Vehicle parameters config ========================= 
				case VC_CFG_PROTOCOL_REG_V_WHEELDIA:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setVehicleParam(VC_PARAM_ID_WHEELD,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_MAXSPEED:
					toSet = rx_frame->Buffer[1] 		  | rx_frame->Buffer[2] << 8 |
									rx_frame->Buffer[3] << 16 | rx_frame->Buffer[3] << 24;
				//	STRG_setVehicleParam(VC_PARAM_ID_MAXSPEED,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_GEARRATIO:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setVehicleParam(VC_PARAM_ID_GEARRATIO,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_THROTTLE_M:
					toSet = rx_frame->Buffer[1];
				//	STRG_setTHParam(VC_PARAM_ID_THROTTLE_M,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_THROTTLE_F:
					toSet = rx_frame->Buffer[1];
				//	STRG_setTHParam(VC_PARAM_ID_THROTTLE_F,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_THROTTLE_OFFSET:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setTHParam(VC_PARAM_ID_THROTTLE_OFFSET,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_THROTTLE_TYPE:
					toSet = rx_frame->Buffer[1];
				//	STRG_setTHParam(VC_PARAM_ID_THROTTLE_TYPE,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_DUALMOTOR:
					toSet = rx_frame->Buffer[1];
				//	STRG_setVehicleParam(VC_PARAM_ID_DUALMOTOR,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_MAINMOTOR:
					toSet = rx_frame->Buffer[1];
				//	STRG_setVehicleParam(VC_PARAM_ID_MAINMOTOR,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_DRT_THRESHOLD:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setDRTparams(toSet, bMotorSelection, true);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_DRT_SLOPE:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setDRTparams(toSet, bMotorSelection, false);
					break;	
				//========================= Motor parameters config ========================= 
				case VC_CFG_PROTOCOL_REG_M_TORQUE_KP:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setMotorParam(MD_PARAM_ID_TORQUE_KP,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_TORQUE_KI:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setMotorParam(MD_PARAM_ID_TORQUE_KI,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FLUX_KP:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setMotorParam(MD_PARAM_ID_FLUX_KP,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FLUX_KI:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setMotorParam(MD_PARAM_ID_FLUX_KI,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_POLEPAIRS:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setMotorParam(MD_PARAM_ID_POLEPAIRS,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_ANGLE_OFFSET:
					// TO DEFINE...
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FF_C1:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
				//	STRG_setMotorParam(MD_PARAM_ID_FF_C1,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FF_C2:
					// TO DEFINE...
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FF_C3:
					// TO DEFINE...
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FW_KP:
					// TO DEFINE...
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FW_KI:
					// TO DEFINE...
					break;
				
				case VC_CFG_PROTOCOL_REG_M_MAXVOLT:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
			//		STRG_setMotorParam(MD_PARAM_ID_MAXVOLT,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_MINVOLT:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
			//		STRG_setMotorParam(MD_PARAM_ID_MINVOLT,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_MAXPHASECURR:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
			//		STRG_setMotorParam(MD_PARAM_ID_MAXPHASECURRENT,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_MAXMOTORTEMP:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
			//		STRG_setMotorParam(MD_PARAM_ID_MAXTEMP,toSet,bMotorSelection);
					break;
				
				default:
					m_host_handle.hError |= HOST_BAD_REG_ID;
					break;				
			}
			// Generate a simple ACK_NOERROR as response
			frame_to_send->Code = ACK_NOERROR;
			frame_to_send->Size = 0;
			
		}break; // End of case VC_PROTOCOL_CODE_SET_REG
		
		default:
			m_host_handle.hError = HOST_BAD_FRAME_ID;
			break;
	} // END of main switch case 
	
	// If there's no error, send an ACK to the host
	if(m_host_handle.hError == HOST_NO_ERROR)
	{
		m_host_handle.frame_handle.TxFrame.FrameCRC =  FCP_CalcCRC(frame_to_send);
		m_host_handle.frame_handle.TxFrameState = FCP_TRANSFER_ONGOING;
		HOST_TX_IRQ_Handler();
	}
}

void HOST_comm_init(VC_Handle_t * pHandle)
{
	m_host_handle.pVController = pHandle;													 // Pointer to Vcontroller
	FCP_Init(&m_host_handle.frame_handle);													 // Init of host frame
	m_host_handle.euart_handler.dev_type = EUART_HOST;				     // Assignation of type
	m_host_handle.euart_handler.p_uart_inst = UART0_INSTANCE_ADDR; // Assignation of UART instance
	m_host_handle.euart_handler.p_evt_handler = HOST_event_handler;
	nrf_drv_uart_config_t uart_host_config =
	{                                                     
    .pseltxd            = UART0_TX_PIN,               
    .pselrxd            = UART0_RX_PIN,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = &m_host_handle.euart_handler,                         
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_115200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	eUART_Init(&m_host_handle.euart_handler, uart_host_config);
	eUART_Receive(&m_host_handle.euart_handler, m_host_handle.euart_handler.rx_byte);
}

