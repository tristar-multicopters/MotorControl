/**
  ******************************************************************************
  * @file    uart_frame_communication_protocol.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module provides uart-specific structures and functions of
	*					 a frame based serial communication protocol. Adapted from STM32 MC SDK.
  *
	******************************************************************************
	*/


/* Includes ------------------------------------------------------------------*/
#include "uart_frame_communication_protocol.h"
#include "board_hardware.h"

/* Private variables ---------------------------------------------------------*/
static const uint16_t UFCP_Usart_Timeout_none  = 0;
static const uint16_t UFCP_Usart_Timeout_start = 1;
static const uint16_t UFCP_Usart_Timeout_stop  = 2;


/* Functions ---------------------------------------------------------*/

void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
	UFCP_Handle_t * pHandle = (UFCP_Handle_t*) p_context;
	
	switch (p_event->type)
	{
			case NRF_DRV_UART_EVT_RX_DONE:
					 if(pHandle->p_uart_inst == UART0_INSTANCE_ADDR) //Check if we are receiving from a bafang screen or not
					{
						if(pHandle->_Super.RxFrame.Size > 123)
						{
						  UFCP_Baf_RX_IRQ_Handler(pHandle, pHandle->rx_buffer[0]);
						}
						else
						{							
						  pHandle->_Super.RxFrame.Size ++;
							nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
						}
					}
					else
					{
					  UFCP_RX_IRQ_Handler(pHandle, pHandle->rx_buffer[0]);
					}
					break;
			
			case NRF_DRV_UART_EVT_ERROR:
					break;
			
			case NRF_DRV_UART_EVT_TX_DONE:
					if(pHandle->p_uart_inst == UART0_INSTANCE_ADDR)
					{
						if(pHandle->_Super.TxFrame.Size > pHandle->_Super.TxFrame.Code)
						{
						  nrf_drv_uart_tx(pHandle->p_uart_inst, &pHandle->_Super.TxFrame.Buffer[pHandle->_Super.TxFrame.Code], 1);
							pHandle->_Super.TxFrame.Code ++;
						}
						else
						{
							nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
						}
						
					}
					else
					{
					  UFCP_TX_IRQ_Handler(pHandle);
					}
					 
					break;
			
			default:
					break;
	}
}

void UFCP_Init( UFCP_Handle_t * pHandle, nrf_drv_uart_config_t uart_config, ufcp_event_handler_t evt_handler)
{

  /* Initialize generic component part */
  FCP_Init( & pHandle->_Super );
	
	pHandle->p_evt_handler = evt_handler;
	
	nrf_drv_uart_init(pHandle->p_uart_inst, &uart_config, uart_event_handler);	
}

/*
 *
 */
void * UFCP_RX_IRQ_Handler( UFCP_Handle_t * pHandle, unsigned short rx_data)
{
  void * ret_val = (void *) & UFCP_Usart_Timeout_none;
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

  if ( FCP_TRANSFER_IDLE != pBaseHandle->RxFrameState )
  {
    uint8_t rx_byte = (uint8_t) rx_data;

    switch ( pBaseHandle->RxFrameLevel )
    {
      case 0: // First Byte received --> The Code
        pBaseHandle->RxFrame.Code = rx_byte;
        /* Need to ask the caller to start our timeout... TODO: Is this really useful? */
        ret_val = (void *) & UFCP_Usart_Timeout_start;

        /* Start Rx Timeout */
        pBaseHandle->RxTimeoutCountdown = pBaseHandle->RxTimeout;
        pBaseHandle->RxFrameLevel++;
			
				nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
        break;

      case 1: // Second Byte received --> Size of the payload
        pBaseHandle->RxFrame.Size = rx_byte;
        pBaseHandle->RxFrameLevel++;
        if ( pBaseHandle->RxFrame.Size >= FCP_MAX_PAYLOAD_SIZE)
        { /* Garbage data received decoded with a payload size that exceeds max*/
          pBaseHandle->RxFrameLevel =0 ;
        }
				nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
        break;

      default: // In the payload or the "CRC"
        if ( pBaseHandle->RxFrameLevel < pBaseHandle->RxFrame.Size + FCP_HEADER_SIZE )
        {
          // read byte is for the payload
          pBaseHandle->RxFrame.Buffer[pBaseHandle->RxFrameLevel - FCP_HEADER_SIZE] = rx_byte;
          pBaseHandle->RxFrameLevel++;
					
					nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
        }
        else
        {
          // read byte is for the "CRC"
          pBaseHandle->RxFrame.FrameCRC = rx_byte;

          /* Need to ask the caller to stop our timeout... TODO: Is this really useful? */
          ret_val = (void *) & UFCP_Usart_Timeout_stop;

          /* Stop Rx Timeout */
          pBaseHandle->RxTimeoutCountdown = 0;
          /* Disable the reception IRQ */
          nrf_drv_uart_rx_disable(pHandle->p_uart_inst);//LL_USART_DisableIT_RXNE(pHandle->USARTx); *********************************
          /* Indicate the reception is complete. */
          pBaseHandle->RxFrameState = FCP_TRANSFER_IDLE;

          /* Check the Control Sum */
          if ( FCP_CalcCRC( & pBaseHandle->RxFrame ) == pBaseHandle->RxFrame.FrameCRC )
          {
            /* OK. the frame is considered correct. Let's forward to client. */
						ufcp_evt_t ufcp_event = {UFCP_FRAME_RECEIVED, pBaseHandle->RxFrame};
						pHandle->p_evt_handler(&ufcp_event);
          }
          else
          {					
            error_code = FCP_MSG_RX_BAD_CRC;
            (void) UFCP_Send( pHandle, FCP_CODE_NACK, & error_code, 1 );
						
						ufcp_evt_t ufcp_event = {UFCP_CRC_ERROR, pBaseHandle->RxFrame};
						pHandle->p_evt_handler(&ufcp_event);
          }
        }
    } /* end of switch ( pBaseHandle->RxFrameLevel ) */
  } /* end of if ( FCP_TRANSFER_IDLE != pBaseHandle->RxFrameState ) */

  return ret_val;
}


/*
 This function takes care of decoding requests received from a bafang display 
*/
void UFCP_Baf_RX_IRQ_Handler(UFCP_Handle_t * pHandle, unsigned short rx_data)
{
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t rx_byte = (uint8_t) rx_data;

	 switch ( pBaseHandle->RxFrameLevel )	
	 {
		 case 0 :		// Command type
		 {
			 if(rx_byte == BAF_CMD_READ || rx_byte == BAF_CMD_WRITE)
			 {
				 pBaseHandle->RxFrame.Code = rx_data;
				 pBaseHandle->RxFrameLevel++;
			 } 
			 nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
		 }
		 break;
			 
		 case 1:  // Data type to read/write
		 case 2:
		 case 3:
		 {
			 if(pBaseHandle->RxFrame.Code == BAF_CMD_READ)
			 {
				 pBaseHandle->RxFrame.Buffer[pBaseHandle->RxFrameLevel-1] = rx_data;
				 if(rx_data != 0x22)
				 {
				  ufcp_evt_t ufcp_event = {UFCP_FRAME_RECEIVED, pBaseHandle->RxFrame};
				  pHandle->p_evt_handler(&ufcp_event); // callback to switch to Bafang module
			   }
				 else //if its 0x22 trash the frame
				 {
				   pBaseHandle->RxFrameLevel = 0;
					 nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
				 }
			 }
			 else // Write code
			 {
				 pBaseHandle->RxFrame.Buffer[pBaseHandle->RxFrameLevel-1] = rx_data;
				 pBaseHandle->RxFrameLevel++;
				 if(pBaseHandle->RxFrame.Buffer[0] == W_ASSIST && pBaseHandle->RxFrameLevel == 3)
				 {
					 ufcp_evt_t ufcp_event = {UFCP_FRAME_RECEIVED, pBaseHandle->RxFrame};
					 pHandle->p_evt_handler(&ufcp_event); // callback to switch to Bafang module
				 }
				 else
					 nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
			 }
		 }
		 break;
		 
		 default:
			 break;
	 }
}
/*
 *
 */
void UFCP_TX_IRQ_Handler(UFCP_Handle_t * pHandle)
{
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;

  if ( FCP_TRANSFER_IDLE != pBaseHandle->TxFrameState )
  {
    uint16_t tx_data;

    switch ( pBaseHandle->TxFrameLevel )
    {
      case 0:
        tx_data = (uint16_t) pBaseHandle->TxFrame.Code;
        break;

      case 1:
        tx_data = (uint16_t) pBaseHandle->TxFrame.Size;
        break;

      default:
        if ( pBaseHandle->TxFrameLevel < pBaseHandle->TxFrame.Size + FCP_HEADER_SIZE )
        {
          tx_data = (uint16_t) pBaseHandle->TxFrame.Buffer[ pBaseHandle->TxFrameLevel - FCP_HEADER_SIZE ];
        }
        else
        {
          tx_data = (uint16_t) pBaseHandle->TxFrame.FrameCRC;
        }
    } /* end of switch ( pBaseHandle->TxFrameLevel ) */

    /* Send the data byte */
		uint8_t tx_data8 = (uint8_t)tx_data;
    nrf_drv_uart_tx(pHandle->p_uart_inst, &tx_data8, 1); //LL_USART_TransmitData8(pHandle->USARTx, tx_data); *********************************

    if ( pBaseHandle->TxFrameLevel < pBaseHandle->TxFrame.Size + FCP_HEADER_SIZE )
    {
      pBaseHandle->TxFrameLevel++;
    }
    else
    {
      //LL_USART_DisableIT_TXE(pHandle->USARTx); ***********************************
      pBaseHandle->TxFrameState = FCP_TRANSFER_IDLE;
			
			ufcp_evt_t ufcp_event = {UFCP_FRAME_SENT, pBaseHandle->TxFrame};
      pHandle->p_evt_handler(&ufcp_event);
    }

  } /* end of if ( FCP_TRANSFER_IDLE != pBaseHandle->TxFrameState ) */
}

/*
 *
 */
void UFCP_OVR_IRQ_Handler(UFCP_Handle_t * pHandle)
{
  uint8_t error_code;

  error_code = UFCP_MSG_OVERRUN;
  (void) UFCP_Send(pHandle, FCP_CODE_NACK, & error_code, 1 );

}

/*
 *
 */
void UFCP_TIMEOUT_IRQ_Handler(UFCP_Handle_t * pHandle)
{
  uint8_t error_code;

  error_code = FCP_MSG_RX_TIMEOUT;
  (void) UFCP_Send(pHandle, FCP_CODE_NACK, & error_code, 1 );

}

uint8_t UFCP_Receive(UFCP_Handle_t * pHandle)
{
	FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t ret_val;

		if(pHandle->p_uart_inst == UART0_INSTANCE_ADDR) // If it's a LCD screen
	{
		  pBaseHandle->RxFrameLevel = 0;
	  	nrf_drv_uart_rx_enable(pHandle->p_uart_inst);
      nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1);
	}
	else
	{
    if ( FCP_TRANSFER_IDLE == pBaseHandle->RxFrameState )
    {
      pBaseHandle->RxFrameLevel = 0;
      pBaseHandle->RxFrameState = FCP_TRANSFER_ONGOING;
	
		  nrf_drv_uart_rx_enable(pHandle->p_uart_inst);
      nrf_drv_uart_rx(pHandle->p_uart_inst, pHandle->rx_buffer, 1); //LL_USART_EnableIT_RXNE(pActualHandle->USARTx); ***************************
      ret_val = FCP_STATUS_WAITING_TRANSFER;
    }
    else
    {
      ret_val = FCP_STATUS_TRANSFER_ONGOING;
    }
  }
  return ret_val;
}

uint8_t UFCP_Send(UFCP_Handle_t * pHandle, uint8_t code, uint8_t *buffer, uint8_t size)
{
	FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t ret_val;

	
		if(pHandle->p_uart_inst == UART0_INSTANCE_ADDR) // If it's a LCD screen
	{
		pBaseHandle->RxFrameLevel = 0;
    pBaseHandle->TxFrame.Code = 1;
		pBaseHandle->TxFrame.Size = size;
		
		uint8_t *dest = pBaseHandle->TxFrame.Buffer;
		while ( size-- ) *dest++ = *buffer++;
		
		nrf_drv_uart_tx(pHandle->p_uart_inst, pBaseHandle->TxFrame.Buffer, 1);
	   

	}
	else
	{
    if ( FCP_TRANSFER_IDLE == pBaseHandle->TxFrameState )
    {
      uint8_t *dest = pBaseHandle->TxFrame.Buffer;

      pBaseHandle->TxFrame.Code = code; 
      pBaseHandle->TxFrame.Size = size;
      while ( size-- ) *dest++ = *buffer++;
      pBaseHandle->TxFrame.FrameCRC = FCP_CalcCRC( & pBaseHandle->TxFrame );

      pBaseHandle->TxFrameLevel = 0;
      pBaseHandle->TxFrameState = FCP_TRANSFER_ONGOING;
		
		  nrf_drv_uart_tx(pHandle->p_uart_inst, & pBaseHandle->TxFrame.Code, 1);
		  //uart_ufcp_inst.uart.p_reg->INTENSET |= UART_INTENSET_TXDRDY_Msk;
      //LL_USART_EnableIT_TXE(pActualHandle->USARTx); ************************
		
		  pBaseHandle->TxFrameLevel++;
		
      ret_val = FCP_STATUS_WAITING_TRANSFER;
    }
    else
    {
      ret_val = FCP_STATUS_TRANSFER_ONGOING;
    }
	}
  return ret_val;
}

void UFCP_AbortReceive( UFCP_Handle_t * pHandle )
{
	FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  pBaseHandle->RxFrameState = FCP_TRANSFER_IDLE;
}

