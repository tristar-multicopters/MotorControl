/**
  ******************************************************************************
  * @file    md_comm.c
  * @author  Sami Bouzid, FTEX
	* @author  Jorge Polo , FTEX
  * @brief   This module handles UART frame-based communication with STM32 Motor Control firmware.
	*					 Functions are provided to send commands to the motor and read its registers.
  *
	******************************************************************************
	*/


#include "md_comm.h"

// Used to enable or disable md_comm debugging flags   
#define MDCOMM_DEBUG_FLAGS   

static MD_Comm_Handle_t * m_pMDcomm;
static UFCP_Handle_t * m_pMDcomm_ufcp;

/************* QUEUES DECLARATIONS **************/
static osMessageQueueId_t pending_frame_queue;
static osMessageQueueId_t transaction_done_queue;

static const osMessageQueueAttr_t queueAtr_pending_frame = {
		.name = "Pending_frame_Q",
};

static const osMessageQueueAttr_t queueAtr_transaction_done = {
		.name = "Transaction_done_Q",
};
/************************************************/

/************* TIMER DECLARATION ****************/
static const osTimerAttr_t TmrAtt_timeout_timer = {
	.name = "MDcomm_TimeOut"
};
static osTimerId_t timeout_timer;
static bool flag_timeout = false;

static void md_timeout_callback( void *argument)
{
	UNUSED_PARAMETER(argument);
	flag_timeout = true;
}	
/************************************************/

static FCP_Frame_t sent_frame;

osThreadId_t TSK_MDcomm_handle;


static void md_frame_event_handler(ufcp_evt_t * p_ufcp_event)
{	
	frame_transaction_t transaction;
	
	switch (p_ufcp_event->evt_type)
	{
			case UFCP_FRAME_RECEIVED:
					m_pMDcomm->hStatus &= ~MDCOMM_TRANSFER_ONGOING;
			
					transaction.tx_frame = sent_frame;
					transaction.rx_frame = p_ufcp_event->frame;
					if ( osMessageQueuePut(transaction_done_queue, &transaction, NULL, 0) == osOK)
					{
						m_pMDcomm->hStatus |= MDCOMM_RXPENDING;
					}
					else
					{
						m_pMDcomm->hError |= MDCOMM_RXBUFFERFULL;
					}
					
					osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
					break;
			
			case UFCP_FRAME_SENT:
					sent_frame = p_ufcp_event->frame;
					break;
			
			case UFCP_CRC_ERROR:
					m_pMDcomm->hStatus &= ~MDCOMM_TRANSFER_ONGOING;
					
			 #ifdef MDCOMM_DEBUG_FLAGS
			    m_pMDcomm->hError |= MDCOMM_BAD_CRC;
				#endif	
			    break;
			
			default:
					break;
	}
}

void md_comm_init(MD_Comm_Handle_t * pMDcomm)
{
	m_pMDcomm = pMDcomm;
	m_pMDcomm_ufcp = & pMDcomm->UARTconfig.ufcp_handle;
	
	nrf_drv_uart_config_t uart_drive_config =
	{                                                     
    .pseltxd            = pMDcomm->UARTconfig.tx_pin,               
    .pselrxd            = pMDcomm->UARTconfig.rx_pin,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = & pMDcomm->UARTconfig.ufcp_handle,                         
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_115200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	UFCP_Init(m_pMDcomm_ufcp, uart_drive_config, md_frame_event_handler);
	
	m_pMDcomm->hError  = MDCOMM_NO_ERROR;
	m_pMDcomm->hStatus = MDCOMM_STANDBY;
}

static void md_frame_received_protocol(FCP_Frame_t * tx_frame, FCP_Frame_t * rx_frame)
{
	uint8_t tx_code = tx_frame->Code;
	uint8_t bMotorSelection = (tx_code & 0xE0) >> 5; /* Mask: 1110|0000 */
	int32_t toSet;
	if ( bMotorSelection != 0 )
	{
		bMotorSelection--;
	}
	tx_code &= 0x1F; /* Mask: 0001|1111 */
	
	switch ( rx_frame->Code )
	{
		case ACK_NOERROR:
			switch (tx_code)
			{
				case MC_PROTOCOL_CODE_EXECUTE_CMD:
					  break;
						
				case MC_PROTOCOL_CODE_GET_REG:
						toSet = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
						
            switch (tx_frame->Buffer[0])
						{							
							case MC_PROTOCOL_REG_BUS_VOLTAGE:
								m_pMDcomm->pMD[bMotorSelection]->MDMeas.bus_voltage_mes = toSet;
								break;
							
							case MC_PROTOCOL_REG_TORQUE_MEAS:
								if (toSet > INT16_MAX)
									m_pMDcomm->pMD[bMotorSelection]->MDMeas.iq_mes = UINT16_MAX-toSet;
								else
									m_pMDcomm->pMD[bMotorSelection]->MDMeas.iq_mes = toSet;
								break;
							
							case MC_PROTOCOL_REG_FLUX_MEAS:
								if (toSet > INT16_MAX)
									m_pMDcomm->pMD[bMotorSelection]->MDMeas.id_mes = UINT16_MAX-toSet;
								else
									m_pMDcomm->pMD[bMotorSelection]->MDMeas.id_mes = toSet;
								break;
							
							case MC_PROTOCOL_REG_HEATS_TEMP:
								m_pMDcomm->pMD[bMotorSelection]->MDMeas.temp_hs = toSet;
								break;
							
							case MC_PROTOCOL_REG_SPEED_MEAS:
								toSet += (rx_frame->Buffer[2] << 16) + (rx_frame->Buffer[3] << 24);
								m_pMDcomm->pMD[bMotorSelection]->MDMeas.speed_mes = toSet;
								break;
							
							case MC_PROTOCOL_REG_FLAGS:
								toSet = toSet | ((rx_frame->Buffer[2] )<< 16 | (rx_frame->Buffer[3] << 24));
								m_pMDcomm->pMD[bMotorSelection]->MDStateMachine.hMFaultOccurred = (toSet & 0xFF) | ((toSet >> 8) & 0xFF);
								m_pMDcomm->pMD[bMotorSelection]->MDStateMachine.hMFaultNow = ((toSet >> 16) & 0xFF) | ((toSet >> 24) & 0xFF);
								break;
							
							case MC_PROTOCOL_REG_STATUS:
								m_pMDcomm->pMD[bMotorSelection]->MDStateMachine.bMState = (MC_State_t)(toSet & 0xFF);
								break;
							
							default:
									break;
						}
						break;	
					case MC_PROTOCOL_CODE_SET_REG:
					case MC_PROTOCOL_CODE_SET_CONFIG_MD:
					default:
						break;
				}
				break;
		case ACK_ERROR:
			#ifdef MDCOMM_DEBUG_FLAGS
				m_pMDcomm->hError |= MDCOMM_ACK_ERROR;
		  #endif
				break;
		default:
			#ifdef MDCOMM_DEBUG_FLAGS  
				m_pMDcomm->hError |= MDCOMM_UNEXPECTED;
		  #endif
				break;
	}
}

/**@brief Task function to send/receive commands/data to motor drive
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
__NO_RETURN void TSK_MDcomm (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
									
	pending_frame_queue = osMessageQueueNew(MD_TXPENDING_BUFFER_SIZE, sizeof(FCP_Frame_t),&queueAtr_pending_frame); 
	transaction_done_queue = osMessageQueueNew(MD_RXPENDING_BUFFER_SIZE, sizeof(frame_transaction_t),&queueAtr_transaction_done);
	
	timeout_timer = osTimerNew(&md_timeout_callback, osTimerOnce, NULL, &TmrAtt_timeout_timer);										
															
	while (true)
	{	
		osThreadFlagsWait(MDCOMM_FLAG, osFlagsWaitAny, 10);
		
		if ( flag_timeout )
		{
			flag_timeout = false;
			if(osTimerIsRunning(timeout_timer))
			{
				osTimerStop(timeout_timer);
			}
			m_pMDcomm->hStatus &= ~MDCOMM_TRANSFER_ONGOING;
			m_pMDcomm->hError |= MDCOMM_TIMEOUT;
		}

		if ( (m_pMDcomm->hStatus & MDCOMM_RXPENDING) )
		{			
			m_pMDcomm->hStatus &= ~MDCOMM_RXPENDING;
			
			if(osTimerIsRunning(timeout_timer))
			{
				osTimerStop(timeout_timer);
			}
			
			frame_transaction_t transaction;
			while (osMessageQueueGet(transaction_done_queue, &transaction, NULL, 0) == osOK)
			{
				md_frame_received_protocol(&transaction.tx_frame, &transaction.rx_frame);
			}
		}

		FCP_Frame_t next_frame;
		if ( !(m_pMDcomm->hStatus & MDCOMM_TRANSFER_ONGOING) )
		{
			if (osMessageQueueGet(pending_frame_queue, &next_frame, NULL, 0) == osOK)
			{
				m_pMDcomm->hStatus |= MDCOMM_TRANSFER_ONGOING;
			
				UFCP_Receive(m_pMDcomm_ufcp);
				UFCP_Send(m_pMDcomm_ufcp, next_frame.Code, next_frame.Buffer, next_frame.Size);
				osTimerStart(timeout_timer, 200);
			}
		}
		if (osMessageQueueGetSpace(pending_frame_queue) == 0)
			m_pMDcomm->hStatus &= ~MDCOMM_TXPENDING;
		else
			m_pMDcomm->hStatus |= MDCOMM_TXPENDING;
	}
}

uint8_t md_startMotor(uint8_t motorSelection)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_EXECUTE_CMD;
	frame.Size = 1;
	frame.Buffer[0] = MC_PROTOCOL_CMD_START_MOTOR;
	if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
		osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint8_t md_stopMotor(uint8_t motorSelection)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_EXECUTE_CMD;
	frame.Size = 1;
	frame.Buffer[0] = MC_PROTOCOL_CMD_STOP_MOTOR;
	if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
		osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint8_t md_faultAcknowledged(uint8_t motorSelection)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_EXECUTE_CMD;
	frame.Size = 1;
	frame.Buffer[0] = MC_PROTOCOL_CMD_FAULT_ACK;
	if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
		osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint8_t md_setTorqueRamp(uint8_t motorSelection, int16_t torque, int16_t duration)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_SET_TORQUE_RAMP;
	frame.Size = 6;
	frame.Buffer[0] = torque & 0xff; // LSB
	frame.Buffer[1] = (torque >> 8) & 0xff; // MSB
	frame.Buffer[4] = duration & 0xff; // LSB
	frame.Buffer[5] = (duration >> 8) & 0xff; // MSB
	if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
		osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint8_t md_setCurrentRef(uint8_t motorSelection, int16_t iq, int16_t id)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_SET_CURRENT_REF;
	frame.Size = 4;
	frame.Buffer[0] = iq & 0xff; // LSB
	frame.Buffer[1] = (iq >> 8) & 0xff; // MSB
	frame.Buffer[2] = id & 0xff; // LSB
	frame.Buffer[3] = (id >> 8) & 0xff; // MSB
	if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
		osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint8_t md_setSpeedRamp(uint8_t motorSelection, int32_t speed, int16_t duration)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_SET_SPEED_RAMP;
	frame.Size = 6;
	frame.Buffer[0] = speed & 0xff; // LSB
	frame.Buffer[1] = (speed >> 8) & 0xff;
	frame.Buffer[2] = (speed >> 16) & 0xff;
	frame.Buffer[3] = (speed >> 24) & 0xff; // MSB
	frame.Buffer[4] = duration & 0xff; // LSB
	frame.Buffer[5] = (duration >> 8) & 0xff; // MSB
	if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
		osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint8_t md_getMDReg(uint8_t motorSelection, MC_Protocol_REG_t reg)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_GET_REG;
	frame.Size = 1;
	frame.Buffer[0] = reg;
	if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
		osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint8_t md_setMDReg(uint8_t motorSelection, MC_Protocol_REG_t reg, int32_t value)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_SET_REG;
	
	switch(reg)
	{
		case MC_PROTOCOL_REG_CONTROL_MODE:
				frame.Size = 2;
				frame.Buffer[0] = reg;
				frame.Buffer[1] = (uint8_t)value;
				if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
					osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
				}
				else {
					m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
					return 0;
				}
				break;		
		
		case MC_PROTOCOL_REG_TORQUE_REF:
		case MC_PROTOCOL_REG_FLUX_REF:
		case MC_PROTOCOL_REG_SPEED_KP:
		case MC_PROTOCOL_REG_SPEED_KI:
		case MC_PROTOCOL_REG_TORQUE_KP:
		case MC_PROTOCOL_REG_TORQUE_KI:
		case MC_PROTOCOL_REG_FLUX_KP:
		case MC_PROTOCOL_REG_FLUX_KI:
				frame.Size = 3;
				frame.Buffer[0] = reg;
				frame.Buffer[1] = value & 0xff; // Lower byte
				frame.Buffer[2] = value >> 8; // Higher byte
				if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
					osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
				}
				else {
					m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
					return 0;
				}
				break;

		case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
				frame.Size = 5;
				frame.Buffer[0] = reg;
				frame.Buffer[1] = value & 0xff; // LSB
				frame.Buffer[2] = (value >> 8) & 0xff;
				frame.Buffer[3] = (value >> 16) & 0xff;
				frame.Buffer[4] = (value >> 24) & 0xff; // MSB
				if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
					osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
				}
				else {
					m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
					return 0;
				}
				break;
		
		default:
			break;
	}
	return 1;
}

uint16_t md_sendMDFrame(FCP_Frame_t frame)
{
	if(osMessageQueuePut(pending_frame_queue, &frame, NULL, 0) == osOK) {
		osThreadFlagsSet(TSK_MDcomm_handle, MDCOMM_FLAG);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

void md_clearError(void)
{
	m_pMDcomm->hError = MDCOMM_NO_ERROR;
}

bool md_isErrorOccured(void)
{
	if ( m_pMDcomm->hError )
	{
		return true;
	}
	return false;
}

bool md_isAckErrorOccured(void)
{
	if ( m_pMDcomm->hError & MDCOMM_ACK_ERROR )
	{
		return true;
	}
	return false;
}

