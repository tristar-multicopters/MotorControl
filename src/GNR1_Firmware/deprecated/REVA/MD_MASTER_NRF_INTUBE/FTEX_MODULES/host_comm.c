/**
  ******************************************************************************
  * @file    host_comm.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles UART frame-based communication with host system (PC, smartphone, ...).
  *
	******************************************************************************
	*/
	
	
#include "host_comm.h"


volatile md_reg_t m_host_reg = {0}; /**< Structure to hold motor registers. */
static volatile md_reg_t * p_host_reg;

extern TaskHandle_t TSK_HOSTreceiveFrames_handle;

	
void host_comm_init(volatile md_reg_t * host_reg)
{
	p_host_reg = host_reg;
	
	//TODO: Initialize communiciation peripherals (USB, Bluetooth, ...) 
}

void TSK_HOSTreceiveFrames (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);											
																		
	while (true)
	{					
		vTaskDelay(100);
	}
}

static void host_frame_received_protocol(FCP_Frame_t * rx_frame)
{
  bool RequireAck = true;
  bool bNoError = false; // Default is error
  uint8_t bErrorCode;
	
	uint8_t * buffer = rx_frame->Buffer;

  switch (rx_frame->Code)
  {
  case MC_PROTOCOL_CODE_SET_REG:
    {
      MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buffer[0];
      bErrorCode = ERROR_CODE_WRONG_SET;

      switch (bRegID)
      {
      case MC_PROTOCOL_REG_CONTROL_MODE:
        {
          /* 8bit variables */
          //bNoError = UI_SetReg(&pHandle->_Super, bRegID, (int32_t)(buffer[1]));
        }
        break;

      case MC_PROTOCOL_REG_TORQUE_REF:
      case MC_PROTOCOL_REG_FLUX_REF:
      case MC_PROTOCOL_REG_SPEED_KP:
      case MC_PROTOCOL_REG_SPEED_KI:
      case MC_PROTOCOL_REG_SPEED_KD:
      case MC_PROTOCOL_REG_TORQUE_KP:
      case MC_PROTOCOL_REG_TORQUE_KI:
      case MC_PROTOCOL_REG_TORQUE_KD:
      case MC_PROTOCOL_REG_FLUX_KP:
      case MC_PROTOCOL_REG_FLUX_KI:
      case MC_PROTOCOL_REG_FLUX_KD:
        {
          /* 16bit variables */
          int32_t wValue = buffer[1] + (buffer[2] << 8);
          //bNoError = UI_SetReg(&pHandle->_Super, bRegID, wValue);
        }
        break;

      case MC_PROTOCOL_REG_FF_1Q:
      case MC_PROTOCOL_REG_FF_1D:
      case MC_PROTOCOL_REG_FF_2:
      case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
        {
          /* 32bit variables */
          int32_t wValue = buffer[1] + (buffer[2] << 8) + (buffer[3] << 16) + (buffer[4] << 24);
          //bNoError = UI_SetReg(&pHandle->_Super, bRegID, wValue);
        }
        break;

      default:
        {
          bErrorCode = ERROR_CODE_SET_READ_ONLY;
        }
        break;
      }
    }
    break;

  case MC_PROTOCOL_CODE_GET_REG:
    {
      MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buffer[0];
      bErrorCode = ERROR_CODE_GET_WRITE_ONLY;

      switch (bRegID)
      {
      case MC_PROTOCOL_REG_STATUS:
      case MC_PROTOCOL_REG_CONTROL_MODE:
        {
          /* 8bit variables */
          //int32_t value = UI_GetReg( &pHandle->_Super, bRegID, &bNoError );
          if ( bNoError == true )
          {
            //pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 1);
            RequireAck = false;
          }
        }
        break;

      case MC_PROTOCOL_REG_SPEED_KP:
      case MC_PROTOCOL_REG_SPEED_KI:
      case MC_PROTOCOL_REG_SPEED_KD:
      case MC_PROTOCOL_REG_TORQUE_REF:
      case MC_PROTOCOL_REG_TORQUE_KP:
      case MC_PROTOCOL_REG_TORQUE_KI:
      case MC_PROTOCOL_REG_TORQUE_KD:
      case MC_PROTOCOL_REG_FLUX_REF:
      case MC_PROTOCOL_REG_FLUX_KP:
      case MC_PROTOCOL_REG_FLUX_KI:
      case MC_PROTOCOL_REG_FLUX_KD:
      case MC_PROTOCOL_REG_BUS_VOLTAGE:
      case MC_PROTOCOL_REG_HEATS_TEMP:
      case MC_PROTOCOL_REG_MOTOR_POWER:
      case MC_PROTOCOL_REG_TORQUE_MEAS:
      case MC_PROTOCOL_REG_FLUX_MEAS:
        {
          //int32_t value = UI_GetReg( &pHandle->_Super, bRegID, &bNoError );
          if ( bNoError == true )
          {
            /* 16bit variables */
            //pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 2);
            RequireAck = false;
          }
        }
        break;

      case MC_PROTOCOL_REG_SPEED_REF:
      case MC_PROTOCOL_REG_SPEED_MEAS:
      case MC_PROTOCOL_REG_FF_1Q:
      case MC_PROTOCOL_REG_FF_1D:
      case MC_PROTOCOL_REG_FF_2:
      case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
        {
          //int32_t value = UI_GetReg( &pHandle->_Super, bRegID, &bNoError);
          if ( bNoError == true )
          {
            /* 32bit variables */
            //pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 4);
            RequireAck = false;
          }
        }
        break;

      default:
        bErrorCode = ERROR_CODE_GET_WRITE_ONLY;
        break;
      }
    }
    break;

  case MC_PROTOCOL_CODE_EXECUTE_CMD:
    {
      uint8_t bCmdID = buffer[0];
      bErrorCode = ERROR_CODE_WRONG_CMD;
      //bNoError = UI_ExecCmd(&pHandle->_Super,bCmdID);
    }
    break;

  default:
    {
      bErrorCode = ERROR_BAD_FRAME_ID;
    }
    break;
  }

  if (RequireAck)
  {
    if (bNoError)
    {
      //pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, NULL, 0);
    }
    else
    {
      //pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
    }
  }
}