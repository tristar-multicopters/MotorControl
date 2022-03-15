/**
  ******************************************************************************
  * @file    mcp2515_comm.c
  * @author  Sami Bouzid, FTEX
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module manages communications with mcp2515 (SPI CAN-controller)
  *
	******************************************************************************
	*/

#include <stdlib.h>
#include "mcp25625_comm.h"

MCP25625_Handle_t * m_pMCPHandle;
NRF_QUEUE_DEF(CAN_Message_t, m_MCP_queue, 32, NRF_QUEUE_MODE_NO_OVERFLOW);
/*******************************************************************************/
static QueueHandle_t CANTX_queue;
TaskHandle_t TSK_CANmsgTX_handle_t;
//********************************************** PRIVATE FUNCTIONS **********************************************//

/**@brief Function for put a received CAN message to the queue
   @p rx_buffer_ID : ID of buffer that was requested, whether RXB0 or RXB1
	 @p data_size    : length of the data message (8 max).
*/

static void MCP25625_CANmsgToQueue(uint8_t rx_buffer_ID)
{
	CAN_Message_t message;
	ret_code_t res;
	uint8_t SID_H = m_pMCPHandle->reg_table[rx_buffer_ID];
	uint8_t SID_L = m_pMCPHandle->reg_table[rx_buffer_ID+1];
	uint8_t EID_8 = m_pMCPHandle->reg_table[rx_buffer_ID+2];
	uint8_t EID_0 = m_pMCPHandle->reg_table[rx_buffer_ID+3];
	uint8_t DLC   = m_pMCPHandle->reg_table[rx_buffer_ID+4];
	uint8_t data_index = rx_buffer_ID+5;
	
	//Recover CAN message ID
	if(SID_L & MCP_EXIDE_MASK) //If ID is extended	
	{
		message.ext = true;
		message.id  =  EID_0 | (EID_8 << 8);
	  message.id |= (SID_L & 0x3) << 16 | ((SID_L & 0xE0) << 13);
		message.id |= (SID_H << 21);
	}
		
	else
	{
		message.ext = false;
		message.id  = (SID_L & 0xE0) >> 5;
		message.id |= (SID_H << 3);
	}
		
	if(DLC & MCP_RTR_MASK) // If Extended frame Remote Transmit Request received
		message.rtr = true;
	else
		message.rtr = false;
	
	//Recover data
	message.length = DLC & 0xF;
	for(int i = 0; i < message.length; i++)
		message.data[i] = m_pMCPHandle->reg_table[data_index+i];
	
	//Put the message in the queue
  res = nrf_queue_push(&m_MCP_queue,&message);
	if(res == NRF_ERROR_NO_MEM)
		return;
}

/**@brief Event handler to managing data coming from SPI. It updates the reg table of the MCP_Handle*/
static void mcp25625_event_handler(void)
{
	uint8_t value = 0;
	uint8_t rx_length = m_pMCPHandle->device.current_transfert.rx_len-1;
	uint8_t command = m_pMCPHandle->device.current_transfert.tx_buffer[0];
	MCP_reg_addr_t address = (MCP_reg_addr_t)m_pMCPHandle->device.current_transfert.tx_buffer[1];
	switch(command)
	{
		case MCP_WRITE:
			value = m_pMCPHandle->device.current_transfert.tx_buffer[2];
			m_pMCPHandle->reg_table[address] = value;
			break;
		case MCP_READ:
			value = m_pMCPHandle->device.current_transfert.rx_buffer[2];
			m_pMCPHandle->reg_table[address] = value;
			break;
		case MCP_READ_RX0:
			{
				value = MCP_RXB0SIDH; // Point to the case of the table register where the RX0 starts
				for(int i = 0; i < rx_length; i++)
				{
					m_pMCPHandle->reg_table[value+i] = m_pMCPHandle->device.current_transfert.rx_buffer[1+i];
				}
				MCP25625_CANmsgToQueue(MCP_RXB0SIDH);
			}
			break;
		case MCP_READ_RX1:
			{
				value = MCP_RXB1SIDH; // Point to the case of the table register where the RX1 starts
				for(int i = 0; i < rx_length ; i++)
				{
					m_pMCPHandle->reg_table[value+i] = m_pMCPHandle->device.current_transfert.rx_buffer[1+i];
				}
				MCP25625_CANmsgToQueue(MCP_RXB1SIDH);
			}
			break;	
		case MCP_RESET:
		case MCP_BITMOD:
		case MCP_READ_STATUS:
			break;

		default:
			return;
	}
}
	

/**@brief Function for write into a register
	 @p p_Handle: Handle of the MCP
   @p address : address register ID that we want to modify
	 @p value: value to be written on the register
*/
static void MCP25625_WriteRegister(MCP25625_Handle_t* p_Handle, MCP_reg_addr_t address, uint8_t value, bool wait)
{
	SPI_Transfer_t transfer = 
	{
		.device_id = p_Handle->device.device_id,
		.tx_buffer = {MCP_WRITE, address, value},
		.tx_len = 3,
		.rx_buffer = NULL,
		.rx_len = 3,
	};
	
	if(wait)
		SPI_Transfer_Wait(p_Handle->pSPI, &transfer);
	else
		SPI_Transfer_IT(p_Handle->pSPI, &transfer);
}


/**@brief Function for read a register
	 @p p_Handle: Handle of the MCP
   @p address : address register ID that we want to read
*/
void MCP25625_ReadRegister(MCP25625_Handle_t* p_Handle, MCP_reg_addr_t address, bool wait)
{
	SPI_Transfer_t transfer = 
	{
		.device_id = p_Handle->device.device_id,
		.tx_buffer = {MCP_READ, address},
		.tx_len = 2,
		.rx_buffer = NULL,
		.rx_len = 3,
	};

	if(wait)
		SPI_Transfer_Wait(p_Handle->pSPI, &transfer);
	else
		SPI_Transfer_IT(p_Handle->pSPI, &transfer);
}


/**@brief Function for modify bits values of a register
	 @p p_Handle: Handle of the MCP
   @p address : address register ID that we want to modify
	 @p value: value to be written on the register
	 @p mask: mask to indicate wich bits we want to modify
*/
static void MCP25625_bitModifyRegister(MCP25625_Handle_t* p_Handle, MCP_reg_addr_t address, uint8_t mask, uint8_t data, bool wait)
{
	SPI_Transfer_t transfer = 
	{
		.device_id = p_Handle->device.device_id,
		.tx_buffer = {MCP_BITMOD, address, mask, data},
		.tx_len = 4,
		.rx_buffer = NULL,
		.rx_len = 4,
	};

	if(wait)
		SPI_Transfer_Wait(p_Handle->pSPI, &transfer);
	else
		SPI_Transfer_IT(p_Handle->pSPI, &transfer);
}


/**@brief Function for Get all data of a Rx buffer
	 @p p_Handle: Handle of the MCP
	 @p buffer_ID: ID of RX buffer (RX0 or RX1)
*/
static void MCP25625_ReadRxBuffer(MCP25625_Handle_t* p_Handle, uint8_t buffer_ID)
{
	SPI_Transfer_t transfer = 
	{
		.device_id = p_Handle->device.device_id,
		.tx_buffer = {buffer_ID},
		.tx_len = 1,
		.rx_buffer = NULL,
		.rx_len = 14,
	};

	SPI_Transfer_Wait(p_Handle->pSPI, &transfer);	
}


/**@brief Function for load a buffer with a CAN message
	 @p p_Handle: Handle of the MCP
	 @p buffer_tx: buffer that contains the CAN message.
*/
static void MCP25625_LoadTxBuffer(MCP25625_Handle_t* p_Handle, uint8_t length, uint8_t *buffer_tx)
{
	SPI_Transfer_t transfer = 
	{
		.device_id = p_Handle->device.device_id,
		.tx_len = length,
		.rx_buffer = NULL,
		.rx_len = 0,
	};
	memcpy(transfer.tx_buffer, buffer_tx, length);  
	SPI_Transfer_Wait(p_Handle->pSPI, &transfer);
}

/**@brief Function for Get all the actual mode of the MCP
	 @p p_Handle: Handle of the MCP
*/
static uint8_t MCP25625_GetMode(MCP25625_Handle_t* p_Handle, bool wait)
{
	MCP25625_ReadRegister(p_Handle, MCP_CANSTAT, wait);
	uint8_t reply = p_Handle->reg_table[MCP_CANSTAT];
		
	return reply & MODE_MASK;
}



/**@brief Function for put the MCP in a new mode (normal, sleep, listening only, config, etc)
	 @p p_Handle: Handle of the MCP
	 @p newmode: The desired mode
*/
static uint8_t MCP25625_RequestNewMode(MCP25625_Handle_t* p_Handle, const uint8_t newmode, bool wait)
{	
	MCP25625_bitModifyRegister(p_Handle, MCP_CANCTRL, MODE_MASK, newmode, wait); 	
	MCP25625_ReadRegister(p_Handle, MCP_CANSTAT, wait);
	uint8_t reply = p_Handle->reg_table[MCP_CANSTAT];
			
	if((reply & MODE_MASK) == newmode)
	{ // We're now in the new mode
		return CAN_OK;
	}
	else
	{
		return CAN_FAILINIT;
	}
}


/**@brief Function for Configurate the baud Rate and the clock of the MCP
	 @p p_Handle: Handle of the MCP
*/
static uint8_t MCP25625_ConfigBaudRate(MCP25625_Handle_t* p_Handle)
{
	uint8_t set, cfg1, cfg2, cfg3;

	set = 1;

	switch (p_Handle->baudrate)
	{
		case (CAN_250KBS):
			cfg1 = MCP_24MHz_250kBPS_CFG1;
			cfg2 = MCP_24MHz_250kBPS_CFG2;
			cfg3 = MCP_24MHz_250kBPS_CFG3;
			break;

		case (CAN_500KBS):
			cfg1 = MCP_20MHz_500kBPS_CFG1;
			cfg2 = MCP_20MHz_500kBPS_CFG2;
			cfg3 = MCP_20MHz_500kBPS_CFG3;
			break;

		case (CAN_1000KBS):
			cfg1 = MCP_20MHz_1000kBPS_CFG1;
			cfg2 = MCP_20MHz_1000kBPS_CFG2;
			cfg3 = MCP_20MHz_1000kBPS_CFG3;
			break;

		default:
			set = 0;
			break;
	 }

	if (set)
	{
		MCP25625_WriteRegister(p_Handle, MCP_CNF1, cfg1, true);
	
		MCP25625_WriteRegister(p_Handle, MCP_CNF2, cfg2, true);
			
		MCP25625_WriteRegister(p_Handle, MCP_CNF3, cfg3, true);
			
		return CAN_OK;
	}
	
	else
	{
		return CAN_FAILINIT;
	}
}


/**@brief Function for reset all the MCP registers and put the MCP in Configuration mode
	 @p p_Handle: Handle of the MCP
*/
static void MCP25625_Reset(MCP25625_Handle_t* p_Handle)
{
	SPI_Transfer_t transfer = 
	{
		.device_id = p_Handle->device.device_id,
		.tx_buffer = {MCP_RESET},
		.tx_len = 1,
		.rx_buffer = NULL,
		.rx_len = 1,
	};

	SPI_Transfer_Wait(p_Handle->pSPI, &transfer);
}



/**@brief Function for Send a CAN message. It is activated when an interrupt is generated by the MCP
	 @p p_Handle: Handle of the MCP
	 @p buffer: ID of TX buffer
*/
static void MCP25625_SendCANmsg(MCP25625_Handle_t* p_Handle, CAN_TX_Buffer_t buffer)
{
	uint8_t REQ_TXn;
	REQ_TXn =  (0x80) | ((0x01) << buffer);
	
	SPI_Transfer_t transfer = 
	{
		.device_id = p_Handle->device.device_id,
		.tx_buffer = {REQ_TXn},
		.tx_len = 1,
		.rx_buffer = NULL,
		.rx_len = 1,
	};
	SPI_Transfer_Wait(p_Handle->pSPI, &transfer);
}


/**@brief Event handler for managing reception and transmission of a CAN message.
	 @p pin: pin input that generates the interrupt
	 @p action: not used
*/
static void MCP25625_IT_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	BaseType_t yield;
	m_pMCPHandle->IT_received = true;
	vTaskNotifyGiveFromISR(TSK_CANmsgTX_handle_t,&yield);
}

/**@brief Function for initialating the input that will be used to cause an interrupt when a 
	        MCP has received data on one of its RX buffers.
*/
static void MCP25625_init_IT(uint8_t interrupt_pin)
{
	nrf_drv_gpiote_in_config_t int_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	int_config.pull = NRF_GPIO_PIN_PULLUP;
	
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(interrupt_pin, &int_config,MCP25625_IT_handler));
	nrf_drv_gpiote_in_event_enable(interrupt_pin, true);
}

//********************************************** PUBLIC FUNCTIONS **********************************************//

void TSK_CANmsg(void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	CAN_toSend_t send_can_message;
	CANTX_queue = xQueueCreate(MCP_CANMSG_SIZE, sizeof(CAN_toSend_t));
	
	while(true)
	{
		if (xTaskNotifyWait(0, 0, NULL, 100))
		{
			// Get Flags...
			MCP25625_ReadRegister(m_pMCPHandle, MCP_CANINTF, true);
			
			if(	m_pMCPHandle->IT_received ) // if an interrupt has been received...
			{
				m_pMCPHandle->IT_received = false;
				if(m_pMCPHandle->reg_table[MCP_CANINTF] & MCP_RX0IF) // Receive message on the RX0 buffer
				{
					// Clear the RX0IF flag to be able to receive futur messages
					MCP25625_bitModifyRegister(m_pMCPHandle, MCP_CANINTF,MCP_RX0IF, 0x0, true);
					//Read Id and data in the RX1 buffer 
					MCP25625_ReadRxBuffer(m_pMCPHandle, MCP_READ_RX0);
				}
				
				else if(m_pMCPHandle->reg_table[MCP_CANINTF] & MCP_RX1IF) // Receive message on the RX1 buffer
				{
					// Clear the RX0IF flag to be able to receive futur messages
					MCP25625_bitModifyRegister(m_pMCPHandle, MCP_CANINTF,MCP_RX1IF, 0x0, true);
					//Read Id and data in the RX1 buffer 
					MCP25625_ReadRxBuffer(m_pMCPHandle, MCP_READ_RX1);
				}
				
				else if(m_pMCPHandle->reg_table[MCP_CANINTF] & MCP_TX0IF) // Transmit a message
				{
					m_pMCPHandle->ongoing_transfer = false;
					MCP25625_bitModifyRegister(m_pMCPHandle, MCP_CANINTF, MCP_TX0_INT, 0x0, true);
				}
				
				else if (m_pMCPHandle->reg_table[MCP_CANINTF] & (MCP_MERRF | MCP_ERRIF))
				{
				  MCP25625_bitModifyRegister(m_pMCPHandle, MCP_CANINTF,
																									 MCP_MERRF | MCP_ERRIF, 0x00, true);
					MCP25625_ReadRegister(m_pMCPHandle, MCP_CANINTF, true);
				}
			 }
			
			if(xQueueReceive(CANTX_queue, & send_can_message, 0) == pdPASS)
			{
				if(!m_pMCPHandle->ongoing_transfer)
				{
					m_pMCPHandle->ongoing_transfer = true;
					MCP25625_LoadTxBuffer(m_pMCPHandle, send_can_message.size, send_can_message.tx_buffer);
					MCP25625_SendCANmsg(m_pMCPHandle, CAN_TX_BUFFER_0);
				}
			}
		}
	}
}		


uint8_t MCP25625_Init(MCP25625_Handle_t* p_Handle)
{	
	uint8_t res;
	m_pMCPHandle = p_Handle;
	
	p_Handle->MCP_queue = &m_MCP_queue;
	p_Handle->IT_received = false;
	p_Handle->ongoing_transfer = false;
	p_Handle->device.pCallback_end = mcp25625_event_handler;
	p_Handle->device.device_id = SPI_AddDevice(p_Handle->pSPI, &p_Handle->device);
	
	// Configure pin CS as output
	nrf_gpio_cfg_output(p_Handle->device.ss_pin);
	nrf_gpio_pin_set(p_Handle->device.ss_pin);
	
	// Use resistor of 120ohms between CANH and CANL
//	nrf_gpio_cfg_output(p_Handle->res120_pin);
//	nrf_gpio_pin_set(p_Handle->res120_pin);
	
	//Put all the registers to their default value and put the MCP in Config mode
	MCP25625_Reset(p_Handle);
	// Check to be sure we're in Config_mode.
	if(MCP25625_GetMode(p_Handle, true) != MODE_CONFIG)
	{
		res = MCP25625_RequestNewMode(p_Handle, MODE_CONFIG, true);
	}
	//Clearing all interrupt flags
	MCP25625_WriteRegister(p_Handle, MCP_CANINTF, 0x00, true);
	
	// Initialise GPIOE for Tx/Rx interrupts
	MCP25625_init_IT(p_Handle->int_pin);
	
	//Set RX0 and TXB0 interruptions
	MCP25625_bitModifyRegister(p_Handle,
														 MCP_CANINTE,
														 MCP_RX_INT | MCP_TX0_INT, 
														 MCP_RX_INT | MCP_TX0_INT, true);
	MCP25625_bitModifyRegister(p_Handle,
														 MCP_CANINTE,
														 MCP_MERRF | MCP_ERRIF,
														 MCP_MERRF | MCP_ERRIF, true);
	MCP25625_ReadRegister(p_Handle, MCP_CANINTE, true);
	// Configure Baud rate
	MCP25625_ConfigBaudRate(p_Handle);
	
	// Accept any message to be save on RX0 buffer and enable rollover on RX1 buffer
	MCP25625_bitModifyRegister(p_Handle, MCP_RXB0CTRL,
                             MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                             MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK, true);
	
	// Accept any message to be save on RX1 buffer
  MCP25625_bitModifyRegister(p_Handle, MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                             MCP_RXB_RX_ANY, true);
	
	// Put MCP on Normal mode
  res = MCP25625_RequestNewMode(p_Handle, MODE_NORMAL, true);		
	return res;
}


void MCP25625_WriteCANmsg(MCP25625_Handle_t* p_Handle, CAN_TX_Buffer_t buffer, CAN_Message_t* pMsg)
{
	MCP_reg_addr_t SIDH_index;
	CAN_toSend_t toSend;
	toSend.size = pMsg->length+6;
	uint8_t tx_size = pMsg->length+6;
	uint32_t id = pMsg->id;
	// Keep the values in the reg table according to the index of the giving TXBuffer
	switch(buffer)
	{
		case CAN_TX_BUFFER_0:
			SIDH_index = MCP_TXB0SIDH;
		  toSend.tx_buffer[0] = MCP_LOAD_TX0;
			break;
		case CAN_TX_BUFFER_1:
			SIDH_index = MCP_TXB1SIDH;
			toSend.tx_buffer[0] = MCP_LOAD_TX1;
			break;
		case CAN_TX_BUFFER_2:
			SIDH_index = MCP_TXB2SIDH;
			toSend.tx_buffer[0] = MCP_LOAD_TX2;
			break;
		default:
			break;
	}
	// Keep in track the reg table indexes
	uint8_t SIDL_index = SIDH_index+1;
	uint8_t EID8_index = SIDH_index+2;
	uint8_t EID0_index = SIDH_index+3;
	uint8_t DLC_index  = SIDH_index+4;
	uint8_t Data_index = SIDH_index+5;
	
	// If ID is extended
	if (pMsg->ext == 1)
		{	
			// Set EID bits (EID0n and EID8n registers)
			p_Handle->reg_table[EID0_index] = (uint8_t)(id & 0xFF);
			p_Handle->reg_table[EID8_index] = (uint8_t)((id >> 8) & 0xFF);
			
			uint16_t hAux = (id >> 16) & 0xFFFF;
			// Set SID[2-0] and EID[17-16] bits on TXBxSIDL register 			
			p_Handle->reg_table[SIDL_index] = (hAux & 0x3) | ((hAux & 0x1C) << 3);
			// Enable Extended identifier
			p_Handle->reg_table[SIDL_index] |= MCP_EXIDE_MASK;
			// Set SID[10-3] bits 
			p_Handle->reg_table[SIDH_index] = (hAux >> 5) & 0xFF;
		}
		
		else
		{ // Set SID[10-3] bits
			p_Handle->reg_table[SIDH_index] = (uint8_t)(id >> 3);
			// Set SID[2-0] and enable Standard identification mode
			p_Handle->reg_table[SIDL_index] = (uint8_t)((id & 0x07) << 5); 
			//Don't use the Extended registers
			p_Handle->reg_table[EID0_index] = 0x0;
			p_Handle->reg_table[EID8_index] = 0x0;		
		}
	// Put the data length with rtr if required		
	p_Handle->reg_table[DLC_index] = pMsg->length | (pMsg->rtr ? MCP_RTR_MASK : 0);
		
	// Put the data on the table register
	for(uint8_t i = 0; i < pMsg->length; i++)
		p_Handle->reg_table[Data_index+i] = pMsg->data[i];		
	
	// Send the CAN message via SPI Bus
	for(uint8_t i = 1; i < tx_size; i++)	
	{
		toSend.tx_buffer[i] = p_Handle->reg_table[SIDH_index+i-1];
	}
	// Notify the CAN task for send the CAN message
	if(xQueueSend(CANTX_queue, &toSend, 0))
	{
		xTaskNotifyGive(TSK_CANmsgTX_handle_t);
	}
}


void MCP25625_popCANmsg( CAN_Message_t * message )
{
	nrf_queue_generic_pop(&m_MCP_queue,message,false);
}
