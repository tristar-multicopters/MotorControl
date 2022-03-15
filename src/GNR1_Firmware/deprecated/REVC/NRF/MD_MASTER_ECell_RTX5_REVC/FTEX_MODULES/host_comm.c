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

static HOST_Comm_Handle_t * m_pHOSTcomm;
static UFCP_Handle_t * m_pHOSTcomm_ufcp;

/************* QUEUES DECLARATIONS **************/
static osMessageQueueId_t pending_frame_queue;

static const osMessageQueueAttr_t queueAtr_pending_frame = {
		.name = "Pending_frame_Q",
};

/************************************************/


static void host_frame_event_handler(ufcp_evt_t * p_ufcp_event)
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

static void host_frame_received_protocol(FCP_Frame_t * rx_frame)
{
	int32_t toSet;
	int32_t toSend;
	FCP_Frame_t replyFrame;
	
	switch ( rx_frame->Code )
	{
		case VC_PROTOCOL_CODE_GET_REG:
				switch (rx_frame->Buffer[0])
				{
					case VC_RT_PROTOCOL_REG_V_FLAGS:
						break;
							
					case VC_RT_PROTOCOL_REG_V_STATUS:
						break;
					
					case VC_RT_PROTOCOL_REG_V_BUS_VOLTAGE:
						toSend = VC_getBattVoltage(m_pHOSTcomm->pVController);
						replyFrame.Code = ACK_NOERROR;
						replyFrame.Size = 2;
						replyFrame.Buffer[0] = toSend & 0xff; // LSB
						replyFrame.Buffer[1] = (toSend >> 8) & 0xff; // MSB
						UFCP_Send(m_pHOSTcomm_ufcp, replyFrame.Code, replyFrame.Buffer, replyFrame.Size);
						break;
				}
	}
}

void host_comm_init(HOST_Comm_Handle_t * pHOSTcomm, VC_Handle_t * pVController)
{
	pHOSTcomm->pVController = pVController;
	m_pHOSTcomm = pHOSTcomm;
	m_pHOSTcomm_ufcp = & pHOSTcomm->UARTconfig.ufcp_handle;
	
	nrf_drv_uart_config_t uart_host_config =
	{                                                     
    .pseltxd            = pHOSTcomm->UARTconfig.tx_pin,               
    .pselrxd            = pHOSTcomm->UARTconfig.rx_pin,              	
    .pselcts            = NRF_UART_PSEL_DISCONNECTED,   
    .pselrts            = NRF_UART_PSEL_DISCONNECTED,   
    .p_context          = & pHOSTcomm->UARTconfig.ufcp_handle,                         
    .hwfc               = NRF_UART_HWFC_DISABLED,       
    .parity             = NRF_UART_PARITY_EXCLUDED,     
    .baudrate           = NRF_UART_BAUDRATE_115200, 		
    .interrupt_priority = 2,                  					
    NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA
	};
	
	UFCP_Init(m_pHOSTcomm_ufcp, uart_host_config, host_frame_event_handler);
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
			host_frame_received_protocol(&rx_frame);
		}
	}
}