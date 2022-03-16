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


volatile md_reg_t m_md_reg = {0}; /**< Structure to hold motor registers. */
static volatile md_reg_t * p_md_reg;

static QueueHandle_t pending_frame_queue;
static QueueHandle_t transaction_done_queue;

static FCP_Frame_t sent_frame;
static bool ongoing_transaction = false;

static TimerHandle_t timeout_timer;
static bool flag_timeout = false;
static bool flag_crc = false;
static bool flag_ackerror = false;

TaskHandle_t TSK_MDreceiveFrames_handle;
TaskHandle_t TSK_MDsendFrames_handle;
TaskHandle_t TSK_THVOLTAGEacquire_handle;

extern int16_t SAADC_Th_value;

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
									case MC_PROTOCOL_REG_TORQUE_KP:
									case MC_PROTOCOL_REG_TORQUE_KI:
									case MC_PROTOCOL_REG_FLUX_KP:
									case MC_PROTOCOL_REG_FLUX_KI:
									case MC_PROTOCOL_REG_SPEED_KP:
									case MC_PROTOCOL_REG_SPEED_KI:
									case MC_PROTOCOL_REG_BUS_VOLTAGE:
									case MC_PROTOCOL_REG_TORQUE_MEAS:
									case MC_PROTOCOL_REG_FLUX_MEAS:
									case MC_PROTOCOL_REG_IQ_SPEEDMODE:
											wval = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8);
											break;
									
									case MC_PROTOCOL_REG_SPEED_MEAS:
									case MC_PROTOCOL_REG_SPEED_REF:
									case MC_PROTOCOL_REG_FLAGS:
											wval = rx_frame->Buffer[0] + (rx_frame->Buffer[1] << 8) +
														(rx_frame->Buffer[2] << 16) + (rx_frame->Buffer[3] << 24);
											break;
									
									case MC_PROTOCOL_REG_CONTROL_MODE:
									case MC_PROTOCOL_REG_STATUS:
											wval = rx_frame->Buffer[0];
											break;
									
									default:
											break;
							}
					}
					setLocalReg((MC_Protocol_REG_t)tx_frame->Buffer[0], wval); // Set value on local register.
					break;
					
			case MC_PROTOCOL_CODE_SET_REG:
			case MC_PROTOCOL_CODE_SET_CONFIG_MD:
			{
				if (rx_frame->Code == ACK_NOERROR)
					{ // Let the TSK_nrfGetData modify the local register
						if(tx_frame->Code == MC_PROTOCOL_CODE_SET_REG)
							regIsBeingSet((MC_Protocol_REG_t)tx_frame->Buffer[0],false);
						else
							configRegIsBeingSet((MC_Protocol_CONFIGREG_t)tx_frame->Buffer[0],false);
					}
					else
						flag_ackerror = false;
					break;
			}
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

void TSK_THVOLTAGEacquire(void * pvParameter)
{
	 UNUSED_PARAMETER(pvParameter);
	while(true)
	{
		switch(p_md_reg->torque_resp)
		{
			case LINEAR_RESP:		//	y = m*x - voffset
				p_md_reg->torque = (int32_t)(SAADC_Th_value*(3600.0/4096.0)*p_md_reg->conf_sens/1000.0) - p_md_reg->conf_voffset;
				break;
				
			case S_RESP:
				p_md_reg->torque = (int32_t)(3000/(1+exp(-0.002*(((int32_t)(SAADC_Th_value*(3600.0/4096.0)*p_md_reg->conf_sens/1000.0))-2000))));
				break;
			default:
				return;
		}
		vTaskDelay(10);
	}
}

int32_t getLocalReg(MC_Protocol_REG_t reg)
{
	int32_t rValue = 0;
	switch(reg)
	{
		case MC_PROTOCOL_REG_FLAGS:
			rValue = p_md_reg->faults;
			break;
		
		case MC_PROTOCOL_REG_STATUS:
			rValue = p_md_reg->state;
			break;
			
		case MC_PROTOCOL_REG_CONTROL_MODE:
			rValue = p_md_reg->mode.value;
			break;
		
		case MC_PROTOCOL_REG_SPEED_KP:
			rValue = p_md_reg->speed_kp.value;
			break;
						
		case MC_PROTOCOL_REG_SPEED_KI:
			rValue = p_md_reg->speed_ki.value;
			break;

		case MC_PROTOCOL_REG_IQ_SPEEDMODE:
			rValue = p_md_reg->speed_Id_ref.value;
			break;
		
		case MC_PROTOCOL_REG_TORQUE_REF:
			rValue = p_md_reg->iq_ref.value;
			break;

		case MC_PROTOCOL_REG_TORQUE_KP:
			rValue = p_md_reg->torque_kp.value;
			break;
		
		case MC_PROTOCOL_REG_TORQUE_KI:
			rValue = p_md_reg->torque_ki.value;
			break;
			
		case MC_PROTOCOL_REG_FLUX_REF:
			rValue = p_md_reg->id_ref.value;
			break;

		case MC_PROTOCOL_REG_FLUX_KP:
			rValue = p_md_reg->flux_kp.value;
			break;
		
		case MC_PROTOCOL_REG_FLUX_KI:
			rValue = p_md_reg->flux_ki.value;
			break;

		case MC_PROTOCOL_REG_BUS_VOLTAGE:
			rValue = p_md_reg->bus_voltage_mes;
			break;
		
		case MC_PROTOCOL_REG_SPEED_REF:
			rValue = p_md_reg->speed_ref.value;
			break;
		
		case MC_PROTOCOL_REG_SPEED_MEAS:
			rValue = p_md_reg->speed_mes;
			break;
		
		case MC_PROTOCOL_REG_TORQUE_MEAS:
			rValue = p_md_reg->iq_mes;
			break;
		
		case MC_PROTOCOL_REG_FLUX_MEAS:
			rValue = p_md_reg->id_mes;
			break;
		
		default:
			break;
	}
	return rValue;
}

void setLocalReg(MC_Protocol_REG_t reg, uint32_t value)
{
	switch(reg)
	{
		case MC_PROTOCOL_REG_CONTROL_MODE:
			if(!p_md_reg->mode.isSetting)
				p_md_reg->mode.value = value;
			break;

		case MC_PROTOCOL_REG_TORQUE_REF:
			if(!p_md_reg->iq_ref.isSetting)
				p_md_reg->iq_ref.value = value;
			break;
		
		case MC_PROTOCOL_REG_FLUX_REF:
			if(!p_md_reg->id_ref.isSetting)
				p_md_reg->id_ref.value = value;
			break;
			
		case MC_PROTOCOL_REG_IQ_SPEEDMODE:
			if(!p_md_reg->speed_Id_ref.isSetting)
				p_md_reg->speed_Id_ref.value = value;
			break;
			
		case MC_PROTOCOL_REG_SPEED_KP:
			if(!p_md_reg->speed_kp.isSetting)
				p_md_reg->speed_kp.value = value;
			break;
		
		case MC_PROTOCOL_REG_SPEED_KI:
			if(!p_md_reg->speed_ki.isSetting)
				p_md_reg->speed_ki.value = value;
			break;	

		case MC_PROTOCOL_REG_TORQUE_KP:
			if(!p_md_reg->torque_kp.isSetting)
				p_md_reg->torque_kp.value = value;
			break;
		
		case MC_PROTOCOL_REG_TORQUE_KI:
			if(!p_md_reg->torque_ki.isSetting)
				p_md_reg->torque_ki.value = value;
			break;		

		case MC_PROTOCOL_REG_FLUX_KP:
			if(!p_md_reg->flux_kp.isSetting)
				p_md_reg->flux_kp.value = value;
			break;
			
		case MC_PROTOCOL_REG_SPEED_REF:
			if(!p_md_reg->speed_ref.isSetting)
				p_md_reg->speed_ref.value = value;
			break;
			
		case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
			if(!p_md_reg->speed_ramp.isSetting)
				p_md_reg->speed_ramp.value = value;
			break;	
		
		case MC_PROTOCOL_REG_FLUX_KI:
			if(!p_md_reg->flux_ki.isSetting)
				p_md_reg->flux_ki.value = value;
			break;	
		
		case MC_PROTOCOL_REG_STATUS:
			p_md_reg->state = value;
			break;
		
		case MC_PROTOCOL_REG_FLAGS:
			p_md_reg->faults = value;
			break;
		
		case MC_PROTOCOL_REG_BUS_VOLTAGE:
			p_md_reg->bus_voltage_mes = value;
			break;
		
		case MC_PROTOCOL_REG_SPEED_MEAS:
			p_md_reg->speed_mes = value;
			break;
		
		case MC_PROTOCOL_REG_TORQUE_MEAS:
			p_md_reg->iq_mes = value;
			break;
		
		case MC_PROTOCOL_REG_FLUX_MEAS:
				p_md_reg->id_mes = value;
				break;
		
		default:
			break;
	}
}

int32_t getLocalConfigReg(MC_Protocol_CONFIGREG_t reg)
{
	int32_t value;
	
	switch(reg)
	{
		case MC_PROTOCOL_CONFIGREG_TORQUE_KP_INIT:
			value = p_md_reg->conf_torque_kp.value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_TORQUE_KI_INIT:
			value = p_md_reg->conf_torque_ki.value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_FLUX_KI_INIT:
		  value = p_md_reg->conf_flux_ki.value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_FLUX_KP_INIT:
			value = p_md_reg->conf_flux_kp.value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_TEMPERATURE_MAX:
			value = p_md_reg->conf_temp_motor_max.value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_MOTOR_PHASE_SHIFT:
			value = p_md_reg->conf_motor_phase_shift.value;
			break;
		
		case MC_PROTOCOL_CONFIGREG_MAX_PHASE_CURRENT:
			value = p_md_reg->conf_max_phase_current.value;
			break;
				
		case MC_PROTOCOL_CONFIGREG_MOTOR_POLE_PAIRS:
			value = p_md_reg->conf_motor_pole_pairs.value;
			break;
		
		case MC_PROTOCOL_CONFIGREG_MAX_DC_VOLTAGE:
			value = p_md_reg->conf_max_dc_volt.value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_MIN_DC_VOLTAGE:
			value = p_md_reg->conf_min_dc_volt.value;
			break;
		
		case MC_PROTOCOL_CONFIGREG_CURRENT_DC:
			value = p_md_reg->conf_iDC.value;
			break;
		
		case MC_PROTOCOL_CONFIGREG_WH_DIAMETER:
			value= p_md_reg->conf_diameter;
			break;
		
		case MC_PROTOCOL_CONFIGREG_WH_MAXSPEED:
			value = p_md_reg->conf_maxSpeed;
			break;
		
		case MC_PROTOCOL_CONFIGREG_TH_VOFFSET:
			value = p_md_reg->conf_voffset;
			break;
		case MC_PROTOCOL_CONFIGREG_TH_V:
			value = p_md_reg->torque;
			break;
		case MC_PROTOCOL_CONFIGREG_TH_SENSIBILITY:
			value = p_md_reg->conf_sens;
			break;
		
		default:
			break;
	}
	return value;
}

void setLocalConfigReg(MC_Protocol_CONFIGREG_t reg, int32_t value)
{
	switch(reg)
	{
		case MC_PROTOCOL_CONFIGREG_TORQUE_KP_INIT:
			if(!p_md_reg->conf_torque_kp.isSetting)
				p_md_reg->conf_torque_kp.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_TORQUE_KI_INIT:
			if(!p_md_reg->conf_torque_ki.isSetting)
				p_md_reg->conf_torque_ki.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_FLUX_KI_INIT:
			if(!p_md_reg->conf_flux_ki.isSetting)
				p_md_reg->conf_flux_ki.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_FLUX_KP_INIT:
			if(!p_md_reg->conf_flux_kp.isSetting)
				p_md_reg->conf_flux_kp.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_TEMPERATURE_MAX:
			if(!p_md_reg->conf_temp_motor_max.isSetting)
				p_md_reg->conf_temp_motor_max.value = value;
				break;
		
				case MC_PROTOCOL_CONFIGREG_MAX_PHASE_CURRENT:
			if(!p_md_reg->conf_max_phase_current.isSetting)
				p_md_reg->conf_max_phase_current.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_MOTOR_PHASE_SHIFT:
			if(!p_md_reg->conf_motor_phase_shift.isSetting)
				p_md_reg->conf_motor_phase_shift.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_MOTOR_POLE_PAIRS:
			if(!p_md_reg->conf_motor_pole_pairs.isSetting)
				p_md_reg->conf_motor_pole_pairs.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_MAX_DC_VOLTAGE:
			if(!p_md_reg->conf_max_dc_volt.isSetting)
				p_md_reg->conf_max_dc_volt.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_MIN_DC_VOLTAGE:
			if(!p_md_reg->conf_min_dc_volt.isSetting)
				p_md_reg->conf_min_dc_volt.value = value;
				break;
			
		case MC_PROTOCOL_CONFIGREG_CURRENT_DC:
			if(!p_md_reg->conf_iDC.isSetting)
				p_md_reg->conf_iDC.value = value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_WH_DIAMETER:
			p_md_reg->conf_diameter= value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_WH_MAXSPEED:
			p_md_reg->conf_maxSpeed = value;
			break;
		
		case MC_PROTOCOL_CONFIGREG_TH_VOFFSET:
			p_md_reg->conf_voffset = value;
			break;
			
		case MC_PROTOCOL_CONFIGREG_TH_SENSIBILITY:
			p_md_reg->conf_sens = value;
			break;
		
		case MC_PROTOCOL_CONFIGREG_TH_TORQUERESP:
			p_md_reg->torque_resp = value;
			
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

void md_sendMDFrame(FCP_Frame_t frame)
{
	xQueueSend(pending_frame_queue, &frame, 0);
  xTaskNotifyGive(TSK_MDsendFrames_handle);
}

void md_getMDReg(MC_Protocol_REG_t reg)
{
	FCP_Frame_t frame;
	frame.Code = MC_PROTOCOL_CODE_GET_REG;
	frame.Size = 1;
	frame.Buffer[0] = reg;
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

void regIsBeingSet(MC_Protocol_REG_t reg, bool isSet)
{
	switch(reg)
	{
		case MC_PROTOCOL_REG_CONTROL_MODE:
			p_md_reg->mode.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_SPEED_KP:
			p_md_reg->speed_kp.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_SPEED_KI:
			p_md_reg->speed_ki.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_IQ_SPEEDMODE:
			p_md_reg->speed_Id_ref.isSetting = isSet;
			break;	
		
		case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
			p_md_reg->speed_ramp.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_TORQUE_REF:
			p_md_reg->iq_ref.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_TORQUE_KP:
			p_md_reg->torque_kp.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_TORQUE_KI:
			p_md_reg->torque_ki.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_FLUX_REF:
			p_md_reg->id_ref.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_FLUX_KP:
			p_md_reg->flux_kp.isSetting = isSet;
			break;
		
		case MC_PROTOCOL_REG_FLUX_KI:
			p_md_reg->flux_ki.isSetting = isSet;
			break;
		
		default:
			break;
	}
}

void configRegIsBeingSet(MC_Protocol_CONFIGREG_t reg, bool isSet)
{
	switch(reg)
	{
		case MC_PROTOCOL_CONFIGREG_TORQUE_KP_INIT:
			p_md_reg->conf_torque_kp.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_TORQUE_KI_INIT:
			p_md_reg->conf_torque_ki.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_FLUX_KI_INIT:
			p_md_reg->conf_flux_ki.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_FLUX_KP_INIT:
			p_md_reg->conf_flux_kp.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_TEMPERATURE_MAX:
			p_md_reg->conf_temp_motor_max.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_MAX_DC_VOLTAGE:
			p_md_reg->conf_max_dc_volt.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_MIN_DC_VOLTAGE:
			p_md_reg->conf_min_dc_volt.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_MAX_PHASE_CURRENT:
			p_md_reg->conf_max_phase_current.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_MOTOR_PHASE_SHIFT:
			p_md_reg->conf_motor_phase_shift.isSetting = isSet;
			break;
			
		case MC_PROTOCOL_CONFIGREG_MOTOR_POLE_PAIRS:
			p_md_reg->conf_motor_pole_pairs.isSetting = isSet;
			break;
			
		default:
			return;
	}
}

bool getFlagAckError( void )
{
	return flag_ackerror;
}

//int32_t getLocalReg(volatile md_reg_t * reg_list, MC_Protocol_REG_t reg)
//{
//	switch(reg)
//	{
//		case MC_PROTOCOL_REG_STATUS:
//				return reg_list->state;
//			
//		case MC_PROTOCOL_REG_CONTROL_MODE:
//				return reg_list->mode;

//		case MC_PROTOCOL_REG_BUS_VOLTAGE:
//				return reg_list->bus_voltage_mes;
//		
//		case MC_PROTOCOL_REG_SPEED_MEAS:
//				return reg_list->speed_mes;
//		
//		case MC_PROTOCOL_REG_TORQUE_MEAS:
//				return reg_list->iq_mes;
//			
//		case MC_PROTOCOL_REG_FLUX_MEAS:
//				return reg_list->id_mes;
//		
//		default:
//			break;
//	}
//	return 0;
//}
