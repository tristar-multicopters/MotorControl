/**
  ******************************************************************************
  * @file    host_comm.h
  * @author  Jorge Andres Polo, FTEX
  * @brief   High level module that controls communication between host and nRF52					 
  *
	******************************************************************************
	*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "md_comm.h"
#include "usbd_comm.h"
#include "vc_config.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HOST_COMM_H
#define __HOST_COMM_H

#define TOTAL_REGISTERS 20
#define CRC_SIZE 1

typedef enum
{
		ACK_ERROR_FROM_STM = 0xFD,
    BAD_FRAME_ID       = 0xFE,
    BAD_FRAME_CRC      = 0xFF
}error_code_t;


static void decode_RX_packet(uint8_t *dataRx);

static void build_TX_packet(uint8_t *dataToSend, uint8_t code, uint8_t size);

static uint16_t data_to_send(uint8_t *tx_buffer, int32_t data, uint8_t size);

static uint8_t buffer_CalcCRC( uint8_t code, uint8_t size, uint8_t* data);

void TSK_ReplyToHost( void *pvParameter);

#endif
