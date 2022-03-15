/**
  ******************************************************************************
  * @file    frame_communication_protocol.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module provides general structures and functions of
	*					 a frame based serial communication protocol. Adapted from STM32 MC SDK.
  *
	******************************************************************************
	*/


/* Includes ------------------------------------------------------------------*/

#include "frame_communication_protocol.h"
 

__weak void FCP_Init( FCP_Handle_t * pHandle )
{
  pHandle->RxTimeoutCountdown = 0;

  pHandle->TxFrame.Code = 0x0;
  pHandle->TxFrame.Size = 0;
  pHandle->TxFrame.FrameCRC = 0;
  pHandle->TxFrameState = FCP_TRANSFER_IDLE;
  pHandle->TxFrameLevel = 0;

  pHandle->RxFrame.Code = 0x0;
  pHandle->RxFrame.Size = 0;
  pHandle->RxFrame.FrameCRC = 0;
  pHandle->RxFrameState = FCP_TRANSFER_IDLE;
  pHandle->RxFrameLevel = 0;
}

__weak void FCP_SetTimeout( FCP_Handle_t * pHandle, uint16_t Timeout )
{
  if ( NULL != pHandle )
  {
    pHandle->RxTimeout = Timeout;
  }
}

__weak uint8_t FCP_CalcCRC( FCP_Frame_t * pFrame )
{
  uint8_t nCRC = 0;
  uint16_t nSum = 0;
  uint8_t idx;

  if( NULL != pFrame )
  {
    nSum += pFrame->Code;
    nSum += pFrame->Size;

    for ( idx = 0; idx < pFrame->Size; idx++ )
    {
      nSum += pFrame->Buffer[idx];
    }

    nCRC = (uint8_t)(nSum & 0xFF) ; // Low Byte of nSum
    nCRC += (uint8_t) (nSum >> 8) ; // High Byte of nSum
  }

  return nCRC ;
}

__weak uint8_t FCP_IsFrameValid( FCP_Frame_t * pFrame )
{
  if ( NULL != pFrame )
    return FCP_CalcCRC(pFrame) == pFrame->Buffer[pFrame->Size];
  else
    return 0;
}

