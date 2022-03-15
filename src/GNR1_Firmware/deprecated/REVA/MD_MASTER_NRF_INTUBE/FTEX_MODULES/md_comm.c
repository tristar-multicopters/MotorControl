/**
  ******************************************************************************
  * @file    md_comm.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles UART frame-based communication with STM32 Motor Control firmware.
	*					 Functions are provided to send commands to the motor and read its registers.
  *
	******************************************************************************
	*/


#include "md_comm.h"


volatile md_reg_t m_md_reg = {0}; /**< Structure to hold motor registers. */
static volatile md_reg_t * p_md_reg;

static QueueHandle_t pending_frame_queue;
static QueueHandle_t transaction_done_queue;

static FCP_Frame_t sent_frame;
static bool ongoing_transaction = false;

static TimerHandle_t timeout_timer;
static bool flag_timeout = false;
static bool flag_crc = false;


static void md_timeout_callback()
{
	xTimerStop(timeout_timer, 0);
	ongoing_transaction = false;
	flag_timeout = true;
}	

static void md_frame_event_handler(ufcp_evt_t * p_ufcp_event)
{	
	BaseType_t yield_req = pdFALSE;
	frame_transaction_t transaction;
	
	switch (p_ufcp_event->evt_type)
	{
			case UFCP_FRAME_RECEIVED:
					//printf("C");
			
					transaction.tx_frame = sent_frame;
					transaction.rx_frame = p_ufcp_event->frame;
					xQueueSendFromISR(transaction_done_queue, &transaction, &yield_req);
					
					ongoing_transaction = false;
					
					vTaskNotifyGiveFromISR(TSK_MDreceiveFrames_handle, &yield_req);
					portYIELD_FROM_ISR(yield_req);
					break;
			
			case UFCP_FRAME_SENT:
					//printf("B");
					
					sent_frame = p_ufcp_event->frame;
					break;
			
			case UFCP_CRC_ERROR:
					xTimerStop(timeout_timer, 0);	
					ongoing_transaction = false;
					flag_crc = true;
					break;
			
			default:
					break;
	}
}

void md_comm_init(volatile md_reg_t * md_reg)
{
	p_md_reg = md_reg;
	
	nrf_drv_uart_config_t uart_drive_config =
	{                                                     
    .pseltxd            = MD_UART_TX_PIN,               
    .pselrxd            = MD_UART_RX_PIN,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = NULL,                         
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_115200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	UFCP_Init(uart_drive_config, md_frame_event_handler);
}


static void md_frame_received_protocol(FCP_Frame_t * tx_frame, FCP_Frame_t * rx_frame)
{
	int32_t wval;
	
	switch (tx_frame->Code)
	{
			case MC_PROTOCOL_CODE_EXECUTE_CMD:
					if (rx_frame->Code == ACK_NOERROR)
					{
					}
					break;
					
			case MC_PROTOCOL_CODE_GET_REG:
					if (rx_frame->Code == ACK_NOERROR)
					{
							switch (tx_frame->Buffer[0])
							{
									case MC_PROTOCOL_REG_BUS_VOLTAGE:
									case MC_PROTOCOL_REG_TORQUE_MEAS:
									case MC_PROTOCOL_REG_FLUX_MEAS:
											wval = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
											setLocalReg(p_md_reg, tx_frame->Buffer[0], wval);
											break;
									
									case MC_PROTOCOL_REG_SPEED_MEAS:
											wval = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8) +
																(rx_frame->Buffer[2] << 16) + (rx_frame->Buffer[3] << 24);
											setLocalReg(p_md_reg, tx_frame->Buffer[0], wval);
											break;
									
									case MC_PROTOCOL_REG_CONTROL_MODE:
									case MC_PROTOCOL_REG_STATUS:
											wval = rx_frame->Buffer[0];
											break;
									
									default:
											break;
							}
					}
					break;
					
			case MC_PROTOCOL_CODE_SET_REG:
					if (rx_frame->Code == ACK_ERROR)
					{
					}
					break;
			
			default:
					break;
	}
}

int32_t getLocalReg(volatile md_reg_t * reg_list, MC_Protocol_REG_t reg)
{
	switch(reg)
	{
		case MC_PROTOCOL_REG_STATUS:
				return reg_list->state;
			
		case MC_PROTOCOL_REG_CONTROL_MODE:
				return reg_list->mode;

		case MC_PROTOCOL_REG_BUS_VOLTAGE:
				return reg_list->bus_voltage_mes;
		
		case MC_PROTOCOL_REG_SPEED_MEAS:
				return reg_list->speed_mes;
		
		case MC_PROTOCOL_REG_TORQUE_MEAS:
				return reg_list->iq_mes;
			
		case MC_PROTOCOL_REG_FLUX_MEAS:
				return reg_list->id_mes;
		
		default:
			break;
	}
	return 0;
}

void setLocalReg(volatile md_reg_t * reg_list, MC_Protocol_REG_t reg, uint32_t value)
{
	switch(reg)
	{
		case MC_PROTOCOL_REG_CONTROL_MODE:
				reg_list->mode = value;
				break;

		case MC_PROTOCOL_REG_TORQUE_REF:
				reg_list->iq_ref = value;
				break;
		
		case MC_PROTOCOL_REG_FLUX_REF:
				reg_list->id_ref = value;
				break;
		
		case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
				reg_list->speed_ref = value;
				break;
			
		case MC_PROTOCOL_REG_SPEED_KP:
				reg_list->speed_kp = value;
				break;
		
		case MC_PROTOCOL_REG_SPEED_KI:
				reg_list->speed_ki = value;
				break;	

		case MC_PROTOCOL_REG_TORQUE_KP:
				reg_list->torque_kp = value;
				break;
		
		case MC_PROTOCOL_REG_TORQUE_KI:
				reg_list->torque_ki = value;
				break;		

		case MC_PROTOCOL_REG_FLUX_KP:
				reg_list->flux_kp = value;
				break;
		
		case MC_PROTOCOL_REG_FLUX_KI:
				reg_list->flux_ki = value;
				break;				
		
		default:
			break;
	}
}

void md_sendCmdToMD(uint32_t cmd)
{
	FCP_Frame_t frame;
	frame.Code = MC_PROTOCOL_CODE_EXECUTE_CMD;
	frame.Size = 1;
	frame.Buffer[0] = cmd;
	xQueueSend(pending_frame_queue, &frame, 0);
	xTaskNotifyGive(TSK_MDsendFrames_handle);
}

void md_setMDReg(MC_Protocol_REG_t reg, int32_t value)
{
	FCP_Frame_t frame;
	frame.Code = MC_PROTOCOL_CODE_SET_REG;
	
	switch(reg)
	{
		case MC_PROTOCOL_REG_CONTROL_MODE:
				frame.Size = 2;
				frame.Buffer[0] = reg;
				frame.Buffer[1] = (uint8_t)value;
				xQueueSend(pending_frame_queue, &frame, 0);
				xTaskNotifyGive(TSK_MDsendFrames_handle);
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
				xQueueSend(pending_frame_queue, &frame, 0);
				xTaskNotifyGive(TSK_MDsendFrames_handle);
				break;

		case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
				frame.Size = 5;
				frame.Buffer[0] = reg;
				frame.Buffer[1] = value & 0xff; // LSB
				frame.Buffer[2] = (value >> 8) & 0xff;
				frame.Buffer[3] = (value >> 16) & 0xff;
				frame.Buffer[4] = (value >> 24) & 0xff; // MSB
				xQueueSend(pending_frame_queue, &frame, 0);
				xTaskNotifyGive(TSK_MDsendFrames_handle);
				break;
		
		default:
			break;
	}
}

void md_getMDReg(MC_Protocol_REG_t reg)
{
	FCP_Frame_t frame;
	frame.Code = MC_PROTOCOL_CODE_GET_REG;
	
	switch(reg)
	{
		case MC_PROTOCOL_REG_BUS_VOLTAGE:
		case MC_PROTOCOL_REG_SPEED_MEAS:
		case MC_PROTOCOL_REG_CONTROL_MODE:
		case MC_PROTOCOL_REG_STATUS:
		case MC_PROTOCOL_REG_TORQUE_MEAS:
		case MC_PROTOCOL_REG_FLUX_MEAS:		
				frame.Size = 1;
				frame.Buffer[0] = reg;
				xQueueSend(pending_frame_queue, &frame, 0);
				xTaskNotifyGive(TSK_MDsendFrames_handle);
				break;
		
		default:
			break;
	}
}

/**@brief Task function to send commands/data to motor drive
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
void TSK_MDsendFrames (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
																					
	pending_frame_queue = xQueueCreate(MD_BUFFER_SIZE, sizeof(FCP_Frame_t));
	timeout_timer = xTimerCreate("Timeout", 200, pdFALSE, 0, md_timeout_callback);
																					
	md_comm_init(&m_md_reg);														
															
	while (true)
	{	
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY))
		{
			FCP_Frame_t next_frame;
			if (ongoing_transaction == false)
			{
				if (xQueueReceive(pending_frame_queue, & next_frame, 0) == pdPASS)
				{
					//printf("A");
					
					ongoing_transaction = true;
				
					UFCP_Receive();
					UFCP_Send(next_frame.Code, next_frame.Buffer, next_frame.Size);
					xTimerStart(timeout_timer, 0);
				}
				else{
					//printf("D");
				}
			}
		}			
	}
}

/**@brief Task function to receive data from motor drive
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
void TSK_MDreceiveFrames (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);			

	transaction_done_queue = xQueueCreate(MD_BUFFER_SIZE, sizeof(frame_transaction_t));
																		
	while (true)
	{					
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY))
		{			
			xTimerStop(timeout_timer, 0);
			
			frame_transaction_t transaction;
			while (xQueueReceive(transaction_done_queue, &transaction, 0) == pdPASS)
			{
				md_frame_received_protocol(&transaction.tx_frame, &transaction.rx_frame);
			}
			
			xTaskNotifyGive(TSK_MDsendFrames_handle);
		}
	}
}
