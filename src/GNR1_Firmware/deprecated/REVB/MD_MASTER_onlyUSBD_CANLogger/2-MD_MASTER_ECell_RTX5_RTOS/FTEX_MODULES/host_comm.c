/**
  ******************************************************************************
  * @file    host_comm.c
  * @author  Jorge Andres Polo, FTEX
  * @brief   High level module that controls communication between the host and 
	*          nRF52 and between nRF52 and STM32					 
  *
	******************************************************************************
	*/
	
#include "host_comm.h"

osThreadId_t TSK_ReplyToHost_handle;

/**@brief Task function that manages data coming from host
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */

void TSK_ReplyToHost( void *pvParameter)
{
	UNUSED_PARAMETER(pvParameter);																		
	while (true)
	{
		osThreadFlagsWait(HOST_FLAG, osFlagsWaitAny, 0);
		
		while (app_usbd_event_queue_process())
		{
				/* Nothing to do */
		}
		
		decode_RX_packet(getRxBuffer());
	}
}

/**@brief Function that manages the data received from the host.
 *
 * @param[in] dataRx: Buffer that contains the received data
 */

static void decode_RX_packet(uint8_t *dataRx)
{
	 bool isError = false;
	 int32_t rValue = 0;
	 uint16_t size = 0;
	 uint8_t dataToSend[FCP_MAX_PAYLOAD_SIZE];
	 FCP_Frame_t rx_frame;
	 rx_frame.Code = dataRx[0];
	 rx_frame.Size = dataRx[1];
	 memcpy(rx_frame.Buffer, dataRx+FCP_HEADER_SIZE, rx_frame.Size);
	 rx_frame.FrameCRC = dataRx[rx_frame.Size+FCP_HEADER_SIZE];
	
	 if(rx_frame.FrameCRC == FCP_CalcCRC(&rx_frame) && !md_isAckErrorOccured())
	 {	
		 switch(rx_frame.Code)
		 {
			 case MC_PROTOCOL_CODE_EXECUTE_CMD:
				 switch(rx_frame.Buffer[0])
				 {
					 case MC_PROTOCOL_CMD_START_MOTOR:
					 case MC_PROTOCOL_CMD_STOP_MOTOR:
					 case MC_PROTOCOL_CMD_FAULT_ACK:
						 md_faultAcknowledge(M1);
						 break;  
					 case MC_PROTOCOL_CMD_ENTER_CONFIG:
					 case MC_PROTOCOL_CMD_OVERWRITE_FLASH:
					 case MC_PROTOCOL_CMD_EXIT_CONFIG:
						 // TO DO: Should rise a flag here to configure the flash memory of nRF
						 break;
					 default:
						 isError = true;
						 break;
				 } 
			 break;
			 
			 case MC_PROTOCOL_CODE_GET_REG:
			 {
				  switch(rx_frame.Buffer[0])
					{
						// Send just 1-byte data
						case MC_PROTOCOL_REG_STATUS:
							break;		
						case MC_PROTOCOL_REG_CONTROL_MODE:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.mode;
						  size = data_to_send(dataToSend, rValue,1);
							break;
						// Send 2-bytes data			
						case MC_PROTOCOL_REG_SPEED_KP:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.speed_kp;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_SPEED_KI:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.speed_ki;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_TORQUE_REF:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.iq_ref;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_TORQUE_KP:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.torque_kp;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_TORQUE_KI:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.torque_ki;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_FLUX_REF:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.id_ref;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_FLUX_KP:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.flux_kp;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_FLUX_KI:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.flux_ki;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_BUS_VOLTAGE:
							rValue = VControl.pMDComm->pMD[M1].MDMeas.bus_voltage_mes;
						  size = data_to_send(dataToSend, rValue,2);
							break;
						case MC_PROTOCOL_REG_HEATS_TEMP:
							break;
						case MC_PROTOCOL_REG_TORQUE_MEAS:
							rValue = VControl.pMDComm->pMD[M1].MDMeas.iq_mes;
						  size = data_to_send(dataToSend, rValue, 2);
							break;
						case MC_PROTOCOL_REG_FLUX_MEAS:
							rValue = VControl.pMDComm->pMD[M1].MDMeas.id_mes;
						  size = data_to_send(dataToSend, rValue, 2);
							break;
						case MC_PROTOCOL_REG_IQ_SPEEDMODE:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.speed_Id_ref;
						  size = data_to_send(dataToSend, rValue, 2);
							break;
						// Send just 4-bytes data
						case MC_PROTOCOL_REG_FLAGS:
							break;
						case MC_PROTOCOL_REG_SPEED_MEAS:
							rValue = VControl.pMDComm->pMD[M1].MDMeas.speed_mes;
						  size = data_to_send(dataToSend, rValue, 4);
							break;
						case MC_PROTOCOL_REG_SPEED_REF:
							rValue = VControl.pMDComm->pMD[M1].MDRTParam.speed_ref;
						  size = data_to_send(dataToSend, rValue, 4);
							break;
						
						default:
							isError = true; // Bad ID get frame received
							break;
				  }
		   } break;
			 
			 case MC_PROTOCOL_CODE_SET_REG:
			 { // TO DO: Determine how to setup the mutex
				 switch(rx_frame.Buffer[0])
				 {
					 case MC_PROTOCOL_REG_CONTROL_MODE:
						 rValue = rx_frame.Buffer[1];
						 VControl.pMDComm->pMD[M1].MDRTParam.mode = rValue;
						 break;
								
					 case MC_PROTOCOL_REG_TORQUE_REF:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.iq_ref = rValue;
						 break;
							
					 case MC_PROTOCOL_REG_FLUX_REF:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.id_ref = rValue;
						 break;
					 
					 case MC_PROTOCOL_REG_IQ_SPEEDMODE:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.speed_Id_ref = rValue;
						 break;
					 
					 case MC_PROTOCOL_REG_SPEED_KP:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.speed_kp = rValue;
						 break;
					 
					 case MC_PROTOCOL_REG_SPEED_KI:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.speed_ki = rValue;
						 break;
					 
					 case MC_PROTOCOL_REG_TORQUE_KP:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.torque_kp = rValue;
						 break;
					 
					 case MC_PROTOCOL_REG_TORQUE_KI:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.torque_ki = rValue;
						 break;
					 
					 case MC_PROTOCOL_REG_FLUX_KP:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.flux_kp = rValue;
						 break;
					 
					 case MC_PROTOCOL_REG_FLUX_KI:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDRTParam.flux_ki = rValue;
						 break;
					 
					 case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
						 rValue = rx_frame.Buffer[1]       | rx_frame.Buffer[2] << 8  |
									    rx_frame.Buffer[3] << 16 | rx_frame.Buffer[4] << 24;
						 VControl.pMDComm->pMD[M1].MDRTParam.speed_ramp = rValue;
						 break;
					 
					 default:
						 break;
					}
				} break;
				 
			 case MC_PROTOCOL_CODE_GET_CONFIG_MD:
			 {
				 switch(rx_frame.Buffer[0])
				 {
					 case MC_PROTOCOL_CONFIGREG_TORQUE_KP_INIT:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.torque_kp;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_TORQUE_KI_INIT:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.torque_ki;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_FLUX_KI_INIT:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.flux_ki;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_FLUX_KP_INIT:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.flux_kp;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_TEMPERATURE_MAX:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.temp_motor_max;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_MAX_DC_VOLTAGE:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.max_dc_volt;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_MIN_DC_VOLTAGE:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.min_dc_volt;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_CURRENT_DC:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.iDC;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_MAX_PHASE_CURRENT:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.max_phase_current;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_MOTOR_PHASE_SHIFT:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.motor_phase_shift;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_MOTOR_POLE_PAIRS:
						 rValue = VControl.pMDComm->pMD[M1].MDConfigParam.motor_pole_pairs;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 
					 case MC_PROTOCOL_CONFIGREG_WH_DIAMETER:
						 rValue = VControl.pVehiculeParam->wheelDiameter;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_WH_MAXSPEED:
						 rValue = VControl.pVehiculeParam->maxSpeed;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_WH_GRATIO:
						 rValue = VControl.pVehiculeParam->gearRatio;
						 size 	= data_to_send(dataToSend, rValue, 2);
						 break;
					 
					 case MC_PROTOCOL_CONFIGREG_TH_TORQUERESP:
						 rValue = VControl.pThrottle->hParam.hOffset;
						 size 	= data_to_send(dataToSend, rValue,1);
						 break;
					 case MC_PROTOCOL_CONFIGREG_TH_VOFFSET:
						 rValue = VControl.pThrottle->hParam.hOffset;
						 size 	= data_to_send(dataToSend, rValue,2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_TH_SENSIBILITY:
						 rValue = VControl.pThrottle->hParam.m;
						 size 	= data_to_send(dataToSend, rValue,2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_TH_SAADC:
						 rValue = VControl.pThrottle->hAvThrottle;
						 size 	= data_to_send(dataToSend, rValue,2);
						 break;
					 case MC_PROTOCOL_CONFIGREG_TH_TORQUE:
						 rValue = VControl.pThrottle->hIqref;
						 size 	= data_to_send(dataToSend, rValue,4); //TO DO: size to verify...
						 break;
					 
					 default:
						 isError = true;
						 break;
				 }
			 } break;
			 
			 case MC_PROTOCOL_CODE_SET_CONFIG_MD:
			 {
				 switch(rx_frame.Buffer[0])
				 {
					 case MC_PROTOCOL_CONFIGREG_TORQUE_KP_INIT:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.torque_kp = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_TORQUE_KI_INIT:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.torque_ki = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_FLUX_KI_INIT:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.flux_ki = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_FLUX_KP_INIT:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.flux_kp = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_TEMPERATURE_MAX:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.temp_motor_max = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_MAX_DC_VOLTAGE:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.max_dc_volt = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_MIN_DC_VOLTAGE:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.min_dc_volt = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_CURRENT_DC:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.iDC = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_MAX_PHASE_CURRENT:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.max_phase_current = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_MOTOR_PHASE_SHIFT:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.motor_phase_shift = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_MOTOR_POLE_PAIRS:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pMDComm->pMD[M1].MDConfigParam.motor_pole_pairs = rValue;
						 break;
					 
					 case MC_PROTOCOL_CONFIGREG_WH_DIAMETER:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pVehiculeParam->wheelDiameter = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_WH_MAXSPEED:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pVehiculeParam->maxSpeed = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_WH_GRATIO:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pVehiculeParam->gearRatio = rValue;
						 break;
						 
					 case MC_PROTOCOL_CONFIGREG_TH_SAADC:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pThrottle->hParam.hMax = rValue;
						 break;

					 case MC_PROTOCOL_CONFIGREG_TH_TORQUERESP:
						 rValue = rx_frame.Buffer[1];
						 VControl.pThrottle->hParam.resp_type = (Throttle_Response_t)rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_TH_VOFFSET:
						 rValue = rx_frame.Buffer[1] | rx_frame.Buffer[2] << 8;
						 VControl.pThrottle->hParam.hOffset = rValue;
						 break;
					 case MC_PROTOCOL_CONFIGREG_TH_SENSIBILITY:
						 
						 rValue = rx_frame.Buffer[1] 			 | rx_frame.Buffer[2] << 8 |
											rx_frame.Buffer[3] << 16 | rx_frame.Buffer[4] << 24;
						 VControl.pThrottle->hParam.F = (rValue >> 16) & 0xFFFF;
						 VControl.pThrottle->hParam.m =  rValue & 0xFFFF;;
						 break;
					 
					 default:
						 isError = true;
						 break;
				 }
			 } break;
			 
			 case MC_PROTOCOL_CODE_SET_CURRENT_REF:
			 {
				 int16_t hIqRef = rx_frame.Buffer[0] + (rx_frame.Buffer[1] << 8);
         int16_t hIdRef = rx_frame.Buffer[2] + (rx_frame.Buffer[3] << 8);
				 VControl.pMDComm->pMD[M1].MDRTParam.iq_ref = hIqRef;
				 VControl.pMDComm->pMD[M1].MDRTParam.id_ref = hIdRef;	 
			 } break;
			 
			 // TO DO: Verify if is worth to keep this command since we'd work in torque mode only
			 case MC_PROTOCOL_CODE_SET_SPEED_RAMP:
			 {
         int32_t speedRPM = rx_frame.Buffer[0] | (rx_frame.Buffer[1] << 8) |
													 (rx_frame.Buffer[2] << 16) | (rx_frame.Buffer[3] << 24);
				 VControl.pMDComm->pMD[M1].MDRTParam.speed_ramp = speedRPM;
			 } break;
				 
			 default:
				 isError = true;
				 break;
		 } // END OF SWITCH CASE
		 
		 if(isError)
		 {
			 dataToSend[0] = (uint8_t)BAD_FRAME_ID;
			 build_TX_packet(dataToSend,ACK_ERROR,1);
		 }
		 else
			 build_TX_packet(dataToSend,ACK_NOERROR,size);
	 }// END OF IF CONDITION
	 
	 else if(md_isAckErrorOccured()) // If STM32 returned an ACKERROR
	 {
		 dataToSend[0] = (uint8_t)ACK_ERROR_FROM_STM;
		 build_TX_packet(dataToSend,ACK_ERROR,1);
	 }
	 else // Bad code CRC, requesting data again...
	 {
		 dataToSend[0] = (uint8_t)BAD_FRAME_CRC;
		 build_TX_packet(dataToSend,ACK_ERROR,1);
	 }
}


/**@brief Private function that builds the tx packet. It adds the header and CRC code to the tx buffer
 *
 * @param[in] dataToSend: Buffer containing the data to send
 * @param[in] code: code of the tx packet
 * @param[in] size: size of the data to send
 */

static void build_TX_packet(uint8_t *dataToSend, uint8_t code, uint8_t size)
{
	uint8_t tx_buffer[FCP_MAX_PAYLOAD_SIZE];
	char index = 0;
	tx_buffer[index++] = code;
	tx_buffer[index++] = size;
	
	if(dataToSend != NULL)
		memcpy(tx_buffer+index,dataToSend,size);
	
	tx_buffer[FCP_HEADER_SIZE+size] = buffer_CalcCRC(code, size, dataToSend);
	USBD_send_data(tx_buffer,FCP_HEADER_SIZE+size+CRC_SIZE);
}

/**@brief Private function for prepare the tx data buffer
 *
 * @param[in] tx_buffer: Buffer containing the data to send
 * @param[in] size: size of packet to send
 * @param[in] data: value to send
 * @param[out] size_2: size of packet to send
 */

static uint16_t data_to_send(uint8_t *tx_buffer, int32_t data, uint8_t size)
{
	int size_2 = size;
	switch(size_2)
	{
		case 1:
			tx_buffer[0] = data & 0xFF;
			break;
		
		case 2:
			tx_buffer[0] =  data & 0xFF;
			tx_buffer[1] = (data >> 8) & 0xFF;
		break;
		
		case 4:
			tx_buffer[0] =  data & 0xFF;
			tx_buffer[1] = (data >> 8) & 0xFF;
			tx_buffer[2] = (data >> 16) & 0xFF;
			tx_buffer[3] = (data >> 24) & 0xFF;
		break;
			
		default:
			tx_buffer = NULL;
			break;
	}
	return size_2;
}

/**@brief Private function for compute CRC code for sending data using USBD or BLE
 *
 * @param[in] code: containts the packet code ID
 * @param[in] size: size of packet to send
 * @param[in] data: buffer that contains the data to send
 * @param[out] nCRC: gives the computed CRC
 */

static uint8_t buffer_CalcCRC( uint8_t code, uint8_t size, uint8_t* data)
{
  uint8_t nCRC = 0;
  uint16_t nSum = 0;
  uint8_t idx;

	nSum += code;
	nSum += size;

	for ( idx = 0; idx < size; idx++ )
	{
		nSum += data[idx];
	}

	nCRC = (uint8_t)(nSum & 0xFF) ; // Low Byte of nSum
	nCRC += (uint8_t) (nSum >> 8) ; // High Byte of nSum

  return nCRC ;
}
