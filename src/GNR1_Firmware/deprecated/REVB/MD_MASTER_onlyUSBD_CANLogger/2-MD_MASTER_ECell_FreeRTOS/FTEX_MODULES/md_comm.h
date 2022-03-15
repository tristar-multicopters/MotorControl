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
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "math.h"

#include "mc_defines.h"
#include "uart_frame_communication_protocol.h"


#define MD_BUFFER_SIZE 	64

#define TIMEOUT_TIMER_INSTANCE	0

#define NB_OF_MOTOR 2

//#define MD_COMM_DEBUG

#ifdef RCM_DEBUG
#include "bsp.h"
#endif

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

typedef struct
{
	FCP_Frame_t tx_frame;
	FCP_Frame_t rx_frame;
} frame_transaction_t;

typedef struct {
	int32_t bus_voltage_mes;
	int32_t speed_mes;
	int32_t iq_mes;
	int32_t id_mes;
	int32_t temp_u;
	int32_t temp_v;
	int32_t temp_motor;
} MD_Measurements_t;

typedef struct {
	int32_t mode;
	int32_t iq_ref;
	int32_t id_ref;
	int32_t speed_ref;
	int32_t speed_ramp;
	int32_t speed_Id_ref;
	int32_t torque_kp;
	int32_t torque_ki;
	int32_t flux_kp;
	int32_t flux_ki;
	int32_t speed_kp;
	int32_t speed_ki;
} MD_RTParameters_t;

typedef struct {
	int32_t torque_kp;
	int32_t torque_ki;
	int32_t flux_kp;
	int32_t flux_ki;
	int32_t temp_motor_max;
	int32_t max_phase_current;
	int32_t motor_phase_shift;
	int32_t motor_pole_pairs;
	int32_t kff;
	int32_t max_dc_volt;
	int32_t min_dc_volt;
	int32_t iDC;
} MD_ConfigParameters_t;

typedef struct
{
	MC_State_t   bMState;          /*!< Variable containing state machine current
                                    state of motor controller */
  uint16_t  hMFaultNow;       /*!< Bit fields variable containing faults
                                    currently present */
  uint16_t  hMFaultOccurred;  /*!< Bit fields variable containing faults
                                    historically occurred since the state
                                    machine has been moved to FAULT_NOW state */
} MDSTM_Handle_t;

typedef struct {
	MD_Measurements_t MDMeas;
	MD_RTParameters_t MDRTParam;
	MD_ConfigParameters_t MDConfigParam;
	MDSTM_Handle_t MDStateMachine;  /*!< State machine handler of motor 1,
																			must be updated regularly */
} MD_Handle_t;

typedef struct {
	nrf_drv_uart_t* p_uart_inst;
	uint32_t tx_pin;
	uint32_t rx_pin;
	
} MD_UART_Config_t;

typedef struct {
	MD_Handle_t pMD[NB_OF_MOTOR];
	MD_UART_Config_t UARTconfig;
	uint16_t hStatus;
	uint16_t hError;
	
} MD_Comm_Handle_t;
	
void md_comm_init(MD_Comm_Handle_t * pMDcomm);

void TSK_MDcomm (void * pvParameter);

uint16_t md_startMotor(uint8_t motorSelection);
uint16_t md_stopMotor(uint8_t motorSelection);
uint16_t md_getMDReg(uint8_t motorSelection, MC_Protocol_REG_t reg);
uint16_t md_setMDReg(uint8_t motorSelection, MC_Protocol_REG_t reg, int32_t value);
uint16_t md_setTorqueRamp(uint8_t motorSelection, int16_t torque, int16_t duration);
uint16_t md_setCurrentRef(uint8_t motorSelection, int16_t iq, int16_t id);
uint16_t md_setSpeedRamp(uint8_t motorSelection, int32_t speed, int16_t duration);
uint16_t md_faultAcknowledge(uint8_t motorSelection);

bool md_isErrorOccured(void);
bool md_isAckErrorOccured(void);

uint16_t md_sendMDFrame(FCP_Frame_t frame);


#endif /*__MD_COMM_H*/
