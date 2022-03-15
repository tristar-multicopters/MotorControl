/**
  ******************************************************************************
  * @file    md_comm.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles UART frame-based communication with STM32 Motor Control firmware.
	*					 Functions are provided to send commands to the motor and read its registers.
  *
	******************************************************************************
	*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MD_COMM_H
#define __MD_COMM_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "md_code_reg.h"
#include "uart_frame_communication_protocol.h"


#define MD_BUFFER_SIZE 	8

#define MD_UART_RX_PIN 			NRF_GPIO_PIN_MAP(0,2)
#define MD_UART_TX_PIN 			NRF_GPIO_PIN_MAP(0,28)

#define TIMEOUT_TIMER_INSTANCE	0

extern TaskHandle_t TSK_MDreceiveFrames_handle;
extern TaskHandle_t TSK_MDsendFrames_handle;

typedef struct
{
	FCP_Frame_t tx_frame;
	FCP_Frame_t rx_frame;
} frame_transaction_t;

typedef struct
{
	int32_t mode;
	int32_t iq_ref;
	int32_t id_ref;
	int32_t speed_ref;
	int32_t torque_kp;
	int32_t torque_ki;
	int32_t flux_kp;
	int32_t flux_ki;
	int32_t speed_kp;
	int32_t speed_ki;
	
	int32_t bus_voltage_mes;
	int32_t speed_mes;
	int32_t iq_mes;
	int32_t id_mes;
	int32_t state;
} md_reg_t;
	
	
void md_comm_init(volatile md_reg_t * md_reg);

void md_sendCmdToMD(uint32_t cmd);
void md_setMDReg(MC_Protocol_REG_t reg, int32_t value);
void md_getMDReg(MC_Protocol_REG_t reg);

void setLocalReg(volatile md_reg_t * reg_list, MC_Protocol_REG_t reg, uint32_t value);
int32_t getLocalReg(volatile md_reg_t * reg_list, MC_Protocol_REG_t reg);

void TSK_MDreceiveFrames (void * pvParameter);
void TSK_MDsendFrames (void * pvParameter);


#endif /*__MD_COMM_H*/
