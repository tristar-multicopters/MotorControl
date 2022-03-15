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

static MD_Comm_Handle_t * m_pMDcomm;

static QueueHandle_t pending_frame_queue;
static QueueHandle_t transaction_done_queue;

static FCP_Frame_t sent_frame;
//static bool ongoing_transaction = false;

static TimerHandle_t timeout_timer;
static bool flag_timeout = false;

TaskHandle_t TSK_MDcomm_handle;


static void md_timeout_callback()
{
	flag_timeout = true;
}	

static void md_frame_event_handler(ufcp_evt_t * p_ufcp_event)
{	
	BaseType_t yield_req = pdFALSE;
	frame_transaction_t transaction;
	
	switch (p_ufcp_event->evt_type)
	{
			case UFCP_FRAME_RECEIVED:
					#ifdef MD_COMM_DEBUG
					bsp_board_led_invert(BSP_BOARD_LED_0);
					//printf("C");
					#endif
			
					transaction.tx_frame = sent_frame;
					transaction.rx_frame = p_ufcp_event->frame;
					if ( xQueueSendFromISR(transaction_done_queue, &transaction, &yield_req) )
					{
						m_pMDcomm->hStatus |= MDCOMM_RXPENDING;
					}
					else
					{
						m_pMDcomm->hError |= MDCOMM_RXBUFFERFULL;
					}
					
					m_pMDcomm->hStatus &= ~MDCOMM_TRANSFER_ONGOING;
					
					vTaskNotifyGiveFromISR(TSK_MDcomm_handle, &yield_req);
					portYIELD_FROM_ISR(yield_req);
					break;
			
			case UFCP_FRAME_SENT:
					#ifdef MD_COMM_DEBUG
					bsp_board_led_invert(BSP_BOARD_LED_1);
					//printf("B");
					#endif
					
					sent_frame = p_ufcp_event->frame;
					break;
			
			case UFCP_CRC_ERROR:
					#ifdef MD_COMM_DEBUG
					bsp_board_led_invert(BSP_BOARD_LED_2);
					#endif
			
					xTimerStop(timeout_timer, 0);	
					m_pMDcomm->hStatus &= ~MDCOMM_TRANSFER_ONGOING;
					m_pMDcomm->hError |= MDCOMM_BAD_CRC;
					break;
			
			default:
					break;
	}
}

void md_comm_init(MD_Comm_Handle_t * pMDcomm)
{
	m_pMDcomm = pMDcomm;
	
	nrf_drv_uart_config_t uart_drive_config =
	{                                                     
    .pseltxd            = pMDcomm->UARTconfig.tx_pin,               
    .pselrxd            = pMDcomm->UARTconfig.rx_pin,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = NULL,                         
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_115200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	#ifdef MD_COMM_DEBUG
	bsp_board_init(BSP_INIT_LEDS);
	#endif
	
	UFCP_Init(pMDcomm->UARTconfig.p_uart_inst, uart_drive_config, md_frame_event_handler);
	
	m_pMDcomm->hError = MDCOMM_NO_ERROR;
	m_pMDcomm->hStatus = MDCOMM_STANDBY;
}

static void md_frame_received_protocol(FCP_Frame_t * tx_frame, FCP_Frame_t * rx_frame)
{
	uint8_t tx_code = tx_frame->Code;
	uint8_t bMotorSelection = (tx_code & 0xE0) >> 5; /* Mask: 1110|0000 */
	if ( bMotorSelection != 0 )
	{
		bMotorSelection--;
	}
	tx_code &= 0x1F; /* Mask: 0001|1111 */
	
	if (rx_frame->Code == ACK_NOERROR)
	{
		switch (tx_code)
		{
			case MC_PROTOCOL_CODE_EXECUTE_CMD:
				break;
					
			case MC_PROTOCOL_CODE_GET_REG:
					switch (tx_frame->Buffer[0])
					{
						case MC_PROTOCOL_REG_TORQUE_KP:
							m_pMDcomm->pMD[bMotorSelection].MDRTParam.torque_kp = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_TORQUE_KI:
							m_pMDcomm->pMD[bMotorSelection].MDRTParam.torque_ki = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_FLUX_KP:
							m_pMDcomm->pMD[bMotorSelection].MDRTParam.flux_kp = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_FLUX_KI:
							m_pMDcomm->pMD[bMotorSelection].MDRTParam.flux_ki = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_SPEED_KP:
							m_pMDcomm->pMD[bMotorSelection].MDRTParam.speed_kp = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_SPEED_KI:
							m_pMDcomm->pMD[bMotorSelection].MDRTParam.speed_ki = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_BUS_VOLTAGE:
							m_pMDcomm->pMD[bMotorSelection].MDMeas.bus_voltage_mes = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_TORQUE_MEAS:
							m_pMDcomm->pMD[bMotorSelection].MDMeas.iq_mes = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_FLUX_MEAS:
							m_pMDcomm->pMD[bMotorSelection].MDMeas.id_mes = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						case MC_PROTOCOL_REG_HEATS_TEMP:
							m_pMDcomm->pMD[bMotorSelection].MDMeas.temp_u = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
							break;
						
						case MC_PROTOCOL_REG_SPEED_MEAS:
							m_pMDcomm->pMD[bMotorSelection].MDMeas.speed_mes = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8) +
										(rx_frame->Buffer[2] << 16) + (rx_frame->Buffer[3] << 24);
							break;
						case MC_PROTOCOL_REG_SPEED_REF:
							m_pMDcomm->pMD[bMotorSelection].MDRTParam.speed_ref = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8) +
										(rx_frame->Buffer[2] << 16) + (rx_frame->Buffer[3] << 24);
							break;
						case MC_PROTOCOL_REG_FLAGS:
							m_pMDcomm->pMD[bMotorSelection].MDStateMachine.hMFaultNow = rx_frame->Buffer[0];
							m_pMDcomm->pMD[bMotorSelection].MDStateMachine.hMFaultOccurred = rx_frame->Buffer[1];
							break;
						
						case MC_PROTOCOL_REG_CONTROL_MODE:
							m_pMDcomm->pMD[bMotorSelection].MDRTParam.mode = rx_frame->Buffer[0];
							break;
						case MC_PROTOCOL_REG_STATUS:
							m_pMDcomm->pMD[bMotorSelection].MDStateMachine.bMState = rx_frame->Buffer[0];
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
	}
//	else
//	{
//		m_pMDcomm->hError |= MDCOMM_ACK_ERROR;
//	}
}

/**@brief Task function to send/receive commands/data to motor drive
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
void TSK_MDcomm (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
																					
	pending_frame_queue = xQueueCreate(MD_BUFFER_SIZE, sizeof(FCP_Frame_t));
	transaction_done_queue = xQueueCreate(MD_BUFFER_SIZE, sizeof(frame_transaction_t));
	timeout_timer = xTimerCreate("Timeout", 200, pdFALSE, 0, md_timeout_callback);														
															
	while (true)
	{	
		xTaskNotifyWait(0, 0, NULL, 10);
			
		if ( flag_timeout )
		{
			flag_timeout = false;
			xTimerStop(timeout_timer, 0);
			m_pMDcomm->hStatus &= ~MDCOMM_TRANSFER_ONGOING;
			m_pMDcomm->hError |= MDCOMM_TIMEOUT;
		}
			
		FCP_Frame_t next_frame;
		if ( !(m_pMDcomm->hStatus & MDCOMM_TRANSFER_ONGOING) )
		{
			if (xQueueReceive(pending_frame_queue, & next_frame, 0) == pdPASS)
			{
				m_pMDcomm->hStatus |= MDCOMM_TRANSFER_ONGOING;
			
				UFCP_Receive();
				UFCP_Send(next_frame.Code, next_frame.Buffer, next_frame.Size);
				xTimerStart(timeout_timer, 0);
			}
		}
		
		if (uxQueueSpacesAvailable(pending_frame_queue) == 0)
			m_pMDcomm->hStatus &= ~MDCOMM_TXPENDING;
		else
			m_pMDcomm->hStatus |= MDCOMM_TXPENDING;

		if ( (m_pMDcomm->hStatus & MDCOMM_RXPENDING) )
		{			
			m_pMDcomm->hStatus &= ~MDCOMM_RXPENDING;
			
			xTimerStop(timeout_timer, 0);
			
			frame_transaction_t transaction;
			while (xQueueReceive(transaction_done_queue, &transaction, 0) == pdPASS)
			{
				md_frame_received_protocol(&transaction.tx_frame, &transaction.rx_frame);
			}
			xTaskNotifyGive(TSK_MDcomm_handle);
		}			
	}
}

uint16_t md_startMotor(uint8_t motorSelection)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_EXECUTE_CMD;
	frame.Size = 1;
	frame.Buffer[0] = MC_PROTOCOL_CMD_START_MOTOR;
	if(xQueueSend(pending_frame_queue, &frame, 0)) {
		xTaskNotifyGive(TSK_MDcomm_handle);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint16_t md_stopMotor(uint8_t motorSelection)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_EXECUTE_CMD;
	frame.Size = 1;
	frame.Buffer[0] = MC_PROTOCOL_CMD_STOP_MOTOR;
	if(xQueueSend(pending_frame_queue, &frame, 0)) {
		xTaskNotifyGive(TSK_MDcomm_handle);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint16_t md_faultAcknowledge(uint8_t motorSelection)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_EXECUTE_CMD;
	frame.Size = 1;
	frame.Buffer[0] = MC_PROTOCOL_CMD_FAULT_ACK;
	if(xQueueSend(pending_frame_queue, &frame, 0)) {
		xTaskNotifyGive(TSK_MDcomm_handle);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint16_t md_setTorqueRamp(uint8_t motorSelection, int16_t torque, int16_t duration)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_SET_TORQUE_RAMP;
	frame.Size = 6;
	frame.Buffer[0] = torque & 0xff; // LSB
	frame.Buffer[1] = (torque >> 8) & 0xff; // MSB
	frame.Buffer[4] = duration & 0xff; // LSB
	frame.Buffer[5] = (duration >> 8) & 0xff; // MSB
	if(xQueueSend(pending_frame_queue, &frame, 0)) {
		xTaskNotifyGive(TSK_MDcomm_handle);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint16_t md_setCurrentRef(uint8_t motorSelection, int16_t iq, int16_t id)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_SET_CURRENT_REF;
	frame.Size = 4;
	frame.Buffer[0] = iq & 0xff; // LSB
	frame.Buffer[1] = (iq >> 8) & 0xff; // MSB
	frame.Buffer[2] = id & 0xff; // LSB
	frame.Buffer[3] = (id >> 8) & 0xff; // MSB
	if(xQueueSend(pending_frame_queue, &frame, 0)) {
		xTaskNotifyGive(TSK_MDcomm_handle);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint16_t md_setSpeedRamp(uint8_t motorSelection, int32_t speed, int16_t duration)
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
	if(xQueueSend(pending_frame_queue, &frame, 0)) {
		xTaskNotifyGive(TSK_MDcomm_handle);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint16_t md_getMDReg(uint8_t motorSelection, MC_Protocol_REG_t reg)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_GET_REG;
	frame.Size = 1;
	frame.Buffer[0] = reg;
	if(xQueueSend(pending_frame_queue, &frame, 0)) {
		xTaskNotifyGive(TSK_MDcomm_handle);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
}

uint16_t md_setMDReg(uint8_t motorSelection, MC_Protocol_REG_t reg, int32_t value)
{
	FCP_Frame_t frame;
	frame.Code = ( (motorSelection+1) << 5 ) + MC_PROTOCOL_CODE_SET_REG;
	
	switch(reg)
	{
		case MC_PROTOCOL_REG_CONTROL_MODE:
				frame.Size = 2;
				frame.Buffer[0] = reg;
				frame.Buffer[1] = (uint8_t)value;
				if(xQueueSend(pending_frame_queue, &frame, 0)) {
					xTaskNotifyGive(TSK_MDcomm_handle);
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
				if(xQueueSend(pending_frame_queue, &frame, 0)) {
					xTaskNotifyGive(TSK_MDcomm_handle);
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
				if(xQueueSend(pending_frame_queue, &frame, 0)) {
					xTaskNotifyGive(TSK_MDcomm_handle);
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
	if(xQueueSend(pending_frame_queue, &frame, 0)) {
		xTaskNotifyGive(TSK_MDcomm_handle);
	}
	else {
		m_pMDcomm->hError |= MDCOMM_TXBUFFERFULL;
		return 0;
	}
	return 1;
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

