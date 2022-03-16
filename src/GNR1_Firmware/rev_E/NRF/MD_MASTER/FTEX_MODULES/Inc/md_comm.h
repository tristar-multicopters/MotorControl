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

#include "math.h"
#include "uart_frame_communication_protocol.h"
#include "mc_defines.h"


#include <cmsis_os2.h>

#define NB_OF_MOTOR 2

#define MD_TXPENDING_BUFFER_SIZE 	32
#define MD_RXPENDING_BUFFER_SIZE 	8

#define TIMEOUT_TIMER_INSTANCE	0

#define  MDCOMM_STANDBY								(uint16_t)(0x0000u)      /**< @brief Nothing to report.*/
#define  MDCOMM_TRANSFER_ONGOING			(uint16_t)(0x0001u)      /**< @brief Transfer currently ongoing.*/
#define  MDCOMM_RXPENDING							(uint16_t)(0x0002u)      /**< @brief Frame present in rx buffer.*/
#define  MDCOMM_TXPENDING							(uint16_t)(0x0004u)      /**< @brief Frame present in tx buffer.*/

#define  MDCOMM_NO_ERROR   						(uint16_t)(0x0000u)     /**< @brief No error.*/
#define  MDCOMM_BAD_CRC  							(uint16_t)(0x0001u)  		/**< @brief Error: STM32 received wrong CRC.*/
#define  MDCOMM_TXBUFFERFULL  				(uint16_t)(0x0002u)     /**< @brief Error: Pending TX frame queue is full.*/
#define	 MDCOMM_TIMEOUT   						(uint16_t)(0x0004u)     /**< @brief Error: No reply from slave.*/
#define	 MDCOMM_ACK_ERROR   					(uint16_t)(0x0008u)     /**< @brief Error: Received ACKERROR from slave.*/
#define  MDCOMM_RXBUFFERFULL  				(uint16_t)(0x0010u)     /**< @brief Error: Pending RX frame queue is full.*/
#define  MDCOMM_UNEXPECTED  					(uint16_t)(0x0020u)     /**< @brief Error: Received unexpected data.*/

#define MDCOMM_FLAG	0x01

typedef struct {
	int32_t bus_voltage_mes;
	int32_t speed_mes;
	int32_t iq_mes;
	int32_t id_mes;
	int32_t temp_hs;
	int32_t temp_motor;
} MD_Measurements_t;

typedef struct
{
	MC_State_t   bMState;       /*!< Variable containing state machine current
                                    state of motor controller */
  uint16_t  hMFaultNow;       /*!< Bit fields variable containing faults
                                    currently present */
  uint16_t  hMFaultOccurred;  /*!< Bit fields variable containing faults
                                    historically occurred since the state
                                    machine has been moved to FAULT_NOW state */
} MDSTM_Handle_t;

typedef struct {
	MD_Measurements_t MDMeas;
	MDSTM_Handle_t MDStateMachine;
} MD_Handle_t;

typedef struct {
	UFCP_Handle_t ufcp_handle;
	uint32_t tx_pin;
	uint32_t rx_pin;
	
} MD_UART_Config_t;

typedef struct {
	MD_Handle_t * pMD[NB_OF_MOTOR];
	MD_UART_Config_t UARTconfig;
	uint16_t hStatus;
	uint16_t hError;
	
} MD_Comm_Handle_t;

typedef struct
{
	FCP_Frame_t tx_frame;
	FCP_Frame_t rx_frame;
} frame_transaction_t;
	
void md_comm_init(MD_Comm_Handle_t * pMDcomm);

__NO_RETURN void TSK_MDcomm (void * pvParameter);

uint8_t md_startMotor(uint8_t motorSelection);
uint8_t md_stopMotor(uint8_t motorSelection);
uint8_t md_getMDReg(uint8_t motorSelection, MC_Protocol_REG_t reg);
uint8_t md_setMDReg(uint8_t motorSelection, MC_Protocol_REG_t reg, int32_t value);
uint8_t md_setTorqueRamp(uint8_t motorSelection, int16_t torque, int16_t duration);
uint8_t md_setCurrentRef(uint8_t motorSelection, int16_t iq, int16_t id);
uint8_t md_setSpeedRamp(uint8_t motorSelection, int32_t speed, int16_t duration);
uint8_t md_faultAcknowledged(uint8_t motorSelection);

void md_clearError(void);
bool md_isErrorOccured(void);
bool md_isAckErrorOccured(void);

uint16_t md_sendMDFrame(FCP_Frame_t frame);


#endif /*__MD_COMM_H*/
