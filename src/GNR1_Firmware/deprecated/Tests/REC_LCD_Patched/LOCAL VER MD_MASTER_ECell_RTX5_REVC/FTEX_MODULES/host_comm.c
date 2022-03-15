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


osThreadId_t TSK_HOSTcomm_handle;
extern osThreadId_t TSK_STRG_handle;

static HOST_Comm_Handle_t * m_pHost;
static UFCP_Handle_t * m_pHOSTcomm_ufcp;

/************* QUEUES DECLARATIONS **************/
static osMessageQueueId_t pending_frame_queue;

static const osMessageQueueAttr_t queueAtr_pending_frame = {
		.name = "Pending_frame_Q",
};

/************************************************/


static void HOST_frame_event_handler(ufcp_evt_t * p_ufcp_event)
{	
	FCP_Frame_t rx_frame = p_ufcp_event->frame;
	
	switch (p_ufcp_event->evt_type)
	{
			case UFCP_FRAME_RECEIVED:
					osMessageQueuePut(pending_frame_queue, &rx_frame, NULL, 0);
					osThreadFlagsSet(TSK_HOSTcomm_handle, HOSTCOMM_FLAG);
					break;
			
			case UFCP_FRAME_SENT:
					break;
			
			case UFCP_CRC_ERROR:
					break;
			
			default:
					break;
	}
}

static void HOST_frame_received_protocol(FCP_Frame_t * rx_frame)
{
	int32_t toSend;
	int32_t toSet;
	FCP_Frame_t replyFrame;
	uint8_t bMotorSelection = (rx_frame->Code & 0xE0) >> 5; /* Mask: 1110|0000 */
	
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
			replyFrame.Code = ACK_NOERROR;
			replyFrame.Size = 1;
			replyFrame.Buffer[0] = toSend & 0xff;
			UFCP_Send(m_pHOSTcomm_ufcp, replyFrame.Code, replyFrame.Buffer, replyFrame.Size);			
		} break;
		
		case VC_PROTOCOL_CODE_EXECUTE_CMD:
		{
			switch(rx_frame->Buffer[0])
			{
				case VC_PROTOCOL_CMD_ENTER_CONFIG:
					/*TO CODE: Indicate to the systeme we're in config mode*/
					break;
				case VC_PROTOCOL_CMD_OVERWRITE_FLASH:
					/*Overwrite nRF flash memory*/
					STRG_setOverWrite(true);
					osThreadFlagsSet(TSK_STRG_handle, STRG_FLAG);
					while(STRG_isOverWritting()); /*To verify*/
					break;
			}
			replyFrame.Code = ACK_NOERROR;
			replyFrame.Size = 0;
			UFCP_Send(m_pHOSTcomm_ufcp, replyFrame.Code, NULL, replyFrame.Size);
			
		}break; // End of case VC_PROTOCOL_CODE_EXECUTE_CMD
			
		case VC_PROTOCOL_CODE_GET_REG:
		{
			switch (rx_frame->Buffer[0])
				{
					case VC_RT_PROTOCOL_REG_V_FLAGS:
						toSend = VC_getVehicleFaultNow(m_pHost->pVController) << 16 |
										 VC_getVehicleFaultOccurred(m_pHost->pVController);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 4;
						replyFrame.Buffer[0] = toSend & 0xff;					// LSB of fault occurred 
						replyFrame.Buffer[1] = (toSend >> 8)  & 0xff;	// MSB of fault occurred
						replyFrame.Buffer[2] = (toSend >> 16) & 0xff; // LSB of fault now
						replyFrame.Buffer[3] = (toSend >> 24) & 0xff;	// MSB of fault now
						break;
					
					case VC_RT_PROTOCOL_REG_V_STATUS:
						toSend = VC_getVehicleState(m_pHost->pVController);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 1;
						replyFrame.Buffer[0] = toSend; 
						break;
					
					case VC_RT_PROTOCOL_REG_V_BUS_VOLTAGE:
						toSend = VC_getBattVoltage(m_pHost->pVController);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff; // LSB
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_V_THROTTLE:
						toSend = VC_getThrottle(m_pHost->pVController);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff;        // LSB
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_V_BRAKE:
						toSend = VC_isBrakeOn(m_pHost->pVController);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 1;
						replyFrame.Buffer[0] = toSend; 
						break;
					
					case VC_RT_PROTOCOL_REG_M_FLAGS:
						toSend = VC_getMotorFaultNow(m_pHost->pVController,bMotorSelection) << 16 |
										 VC_getMotorFaultOccurred(m_pHost->pVController,bMotorSelection);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 4;
						replyFrame.Buffer[0] = toSend & 0xff;					// LSB of fault occurred 
						replyFrame.Buffer[1] = (toSend >> 8)  & 0xff;	// MSB of fault occurred
						replyFrame.Buffer[2] = (toSend >> 16) & 0xff; // LSB of fault now
						replyFrame.Buffer[3] = (toSend >> 24) & 0xff;	// MSB of fault now
						break;
					
					case VC_RT_PROTOCOL_REG_M_STATUS:
						toSend = VC_getMotorState(m_pHost->pVController, bMotorSelection);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 1;
						replyFrame.Buffer[0] = toSend;
						break;
					
					case VC_RT_PROTOCOL_REG_M_TORQUE_KP:
						toSend = VC_getMotorTorqueKp(m_pHost->pVController, bMotorSelection);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff;					// LSB of torque Kp
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff;	// MSB of torque kp
						break;
					
					case VC_RT_PROTOCOL_REG_M_TORQUE_KI:
						toSend = VC_getMotorTorqueKi(m_pHost->pVController, bMotorSelection);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff;					// LSB of torque Ki
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff;	// MSB of torque Ki
						break;
					
					case VC_RT_PROTOCOL_REG_M_FLUX_KP:
						toSend = VC_getMotorFluxKp(m_pHost->pVController, bMotorSelection);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff;					// LSB of flux Kp
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff;	// MSB of flux Kp
						break;
					
					case VC_RT_PROTOCOL_REG_M_FLUX_KI:
						toSend = VC_getMotorFluxKi(m_pHost->pVController, bMotorSelection);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff;					// LSB of flux Ki
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff;	// MSB of flux Ki
						break;
					
					case VC_RT_PROTOCOL_REG_M_HEATS_TEMP:
						toSend = VC_getMotorTemp(m_pHost->pVController, bMotorSelection);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff; 			 // LSB
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_M_TORQUE_MEAS:
						toSend = VC_getMotorIqIdMeas(m_pHost->pVController, bMotorSelection).q;
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff;        // LSB
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_M_FLUX_MEAS:
						toSend = VC_getMotorIqIdMeas(m_pHost->pVController, bMotorSelection).d;
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff;        // LSB
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff; // MSB
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_A:
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_B:
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_Q:
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_D:
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_Q_REF:
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_I_D_REF:
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_V_Q:
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_V_D:
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 0;
						break;
					
					case VC_RT_PROTOCOL_REG_M_MEAS_ROT_SPEED:
						toSend = VC_getMotorSpeedMeas(m_pHost->pVController, bMotorSelection);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 4;
						replyFrame.Buffer[0] = toSend & 0xff;					// LSB of Speed 
						replyFrame.Buffer[1] = (toSend >> 8)  & 0xff;	
						replyFrame.Buffer[2] = (toSend >> 16) & 0xff; 
						replyFrame.Buffer[3] = (toSend >> 24) & 0xff;	// MSB of Speed
						break;
					
					case VC_CFG_PROTOCOL_REG_V_DUALMOTOR:
				#if IS_DUAL
						toSend = DUAL_M;
				#else
						toSend = SINGLE_M;
				#endif
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 1;
						replyFrame.Buffer[0] = toSend;
					default:
						break;
				}
			// Send an answer to the host	
			UFCP_Send(m_pHOSTcomm_ufcp, replyFrame.Code, replyFrame.Buffer, replyFrame.Size);	
		}break; // End of case VC_PROTOCOL_CODE_GET_REG
		
		case VC_PROTOCOL_CODE_SET_REG:
		{
			switch(rx_frame->Buffer[0])
			{
				/************************** Vehicle parameters config ***************************/
				case VC_CFG_PROTOCOL_REG_V_WHEELDIA:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setVehicleParam(VC_PARAM_ID_WHEELD,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_MAXSPEED:
					toSet = rx_frame->Buffer[1] 		  | rx_frame->Buffer[2] << 8 |
									rx_frame->Buffer[3] << 16 | rx_frame->Buffer[3] << 24;
					STRG_setVehicleParam(VC_PARAM_ID_MAXSPEED,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_GEARRATIO:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setVehicleParam(VC_PARAM_ID_GEARRATIO,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_THROTTLE_M:
					toSet = rx_frame->Buffer[1];
					STRG_setTHParam(VC_PARAM_ID_THROTTLE_M,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_THROTTLE_F:
					toSet = rx_frame->Buffer[1];
					STRG_setTHParam(VC_PARAM_ID_THROTTLE_F,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_THROTTLE_OFFSET:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setTHParam(VC_PARAM_ID_THROTTLE_OFFSET,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_THROTTLE_TYPE:
					toSet = rx_frame->Buffer[1];
					STRG_setTHParam(VC_PARAM_ID_THROTTLE_TYPE,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_DUALMOTOR:
					toSet = rx_frame->Buffer[1];
					STRG_setVehicleParam(VC_PARAM_ID_DUALMOTOR,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_MAINMOTOR:
					toSet = rx_frame->Buffer[1];
					STRG_setVehicleParam(VC_PARAM_ID_MAINMOTOR,toSet);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_DRT_THRESHOLD:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setDRTparams(toSet, bMotorSelection, true);
					break;
				
				case VC_CFG_PROTOCOL_REG_V_DRT_SLOPE:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setDRTparams(toSet, bMotorSelection, false);
					break;	
				/************************** Motor parameters config ***************************/
				case VC_CFG_PROTOCOL_REG_M_TORQUE_KP:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_TORQUE_KP,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_TORQUE_KI:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_TORQUE_KI,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FLUX_KP:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_FLUX_KP,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FLUX_KI:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_FLUX_KI,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_POLEPAIRS:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_POLEPAIRS,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_ANGLE_OFFSET:
					/* TO DEFINE...*/
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FF_C1:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_FF_C1,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FF_C2:
					/* TO DEFINE...*/
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FF_C3:
					/* TO DEFINE...*/
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FW_KP:
					/* TO DEFINE...*/
					break;
				
				case VC_CFG_PROTOCOL_REG_M_FW_KI:
					/* TO DEFINE...*/
					break;
				
				case VC_CFG_PROTOCOL_REG_M_MAXVOLT:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_MAXVOLT,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_MINVOLT:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_MINVOLT,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_MAXPHASECURR:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_MAXPHASECURRENT,toSet,bMotorSelection);
					break;
				
				case VC_CFG_PROTOCOL_REG_M_MAXMOTORTEMP:
					toSet = rx_frame->Buffer[1] | rx_frame->Buffer[2] << 8;
					STRG_setMotorParam(MD_PARAM_ID_MAXTEMP,toSet,bMotorSelection);
					break;
				
				default:
					break;				
			}
			// Generate a simple ACK_NOERROR as response
			replyFrame.Code = ACK_NOERROR;
			replyFrame.Size = 0;
			UFCP_Send(m_pHOSTcomm_ufcp, replyFrame.Code, NULL, replyFrame.Size);
		}break; // End of case VC_PROTOCOL_CODE_SET_REG
	} // END of main switch case 
}

void HOST_comm_init(HOST_Comm_Handle_t * pHOSTcomm)
{
	m_pHost = pHOSTcomm;
	m_pHOSTcomm_ufcp = &pHOSTcomm->UARTconfig.ufcp_handle;
	
	nrf_drv_uart_config_t uart_host_config =
	{                                                     
    .pseltxd            = pHOSTcomm->UARTconfig.tx_pin,               
    .pselrxd            = pHOSTcomm->UARTconfig.rx_pin,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = &pHOSTcomm->UARTconfig.ufcp_handle,                         
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_115200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	UFCP_Init(m_pHOSTcomm_ufcp, uart_host_config, HOST_frame_event_handler);
}


/**@brief Task function to send/receive commands/data to/from host
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
__NO_RETURN void TSK_HOSTcomm (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	pending_frame_queue = osMessageQueueNew(HOST_PENDING_BUFFER_SIZE, sizeof(FCP_Frame_t),&queueAtr_pending_frame);
	FCP_Frame_t rx_frame;
	
	while (true)
	{
		UFCP_Receive(m_pHOSTcomm_ufcp);
		
		osThreadFlagsWait(HOSTCOMM_FLAG, osFlagsWaitAny, osWaitForever);
		
		if ( osMessageQueueGet(pending_frame_queue, &rx_frame, NULL, 0) == osOK )
		{
			HOST_frame_received_protocol(&rx_frame);
		}
	}
}
