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

#include "bsp.h"
#include "app_button.h"

#include "md_code_reg.h"
#include "uart_frame_communication_protocol.h"
#include "nrfx_timer.h"
#include "app_uart.h"


#define MD_BUFFER_SIZE 	64
//#define MD_NB_REGS		  16

#define MD_UART_RX_PIN 			NRF_GPIO_PIN_MAP(0,2)
#define MD_UART_TX_PIN 			NRF_GPIO_PIN_MAP(0,28)

#define TIMEOUT_TIMER_INSTANCE	0

//typedef enum 
//{
//	REG_MODE,
//	REG_IQ_REF,
//	REG_ID_REF,
//	REG_SPEED_REF,
//	REG_TORQUE_KP,
//	REG_TORQUE_KI,
//	REG_FLLUX_KP,
//	REG_FLUX_KI,
//	REG_SPEED_KP,
//	REG_SPEED_KI,
//	REG_BUS_VOLTAGE_MES,
//	REG_SPEED_MES,
//	REG_IQ_MES,
//	REG_ID_MES,
//	REG_STATE,
//	REG_FAULTS
//} regs_ID_t;

typedef struct
{
	FCP_Frame_t tx_frame;
	FCP_Frame_t rx_frame;
} frame_transaction_t;

typedef struct
{
	int32_t 		    value;
	bool 				isSetting;
} value_reg_t;

//typedef nrfToSTM_reg_t md_reg_t[MD_NB_REGS];

typedef struct
{
	value_reg_t mode;
	value_reg_t iq_ref;
	value_reg_t id_ref;
	value_reg_t speed_ref;
	value_reg_t speed_ramp;
	value_reg_t speed_Id_ref;
	value_reg_t torque_kp;
	value_reg_t torque_ki;
	value_reg_t flux_kp;
	value_reg_t flux_ki;
	value_reg_t speed_kp;
	value_reg_t speed_ki;
	
	int32_t bus_voltage_mes;
	int32_t speed_mes;
	int32_t iq_mes;
	int32_t id_mes;
	int32_t state;
	int32_t faults;
	
	/*Config registers*/
	// Motor Setup
	value_reg_t conf_torque_kp;
	value_reg_t conf_torque_ki;
	value_reg_t conf_flux_kp;
	value_reg_t conf_flux_ki;
	value_reg_t conf_temp_motor_max;
	value_reg_t conf_max_phase_current;
	value_reg_t conf_motor_phase_shift;
	value_reg_t conf_motor_pole_pairs;
	value_reg_t conf_kff;
	// Power Setup
	value_reg_t conf_max_dc_volt;
	value_reg_t conf_min_dc_volt;
	value_reg_t conf_iDC;
	// Throttle Setup
	MD_torqueResponse_t torque_resp;
	int32_t torque;
	int32_t conf_voffset;
	int16_t conf_sens;
	// Wheel Setup
	int32_t conf_diameter;
	int32_t conf_maxSpeed;
	int32_t gearRatio;
} md_reg_t;
	
void md_comm_init(volatile md_reg_t * md_reg);

void TSK_MDreceiveFrames (void * pvParameter);
void TSK_MDsendFrames (void * pvParameter);
void TSK_THVOLTAGEacquire(void * pvParameter);

void md_sendCmdToMD(uint32_t cmd);
void md_sendMDFrame(FCP_Frame_t frame);
void md_getMDReg(MC_Protocol_REG_t reg);

void setLocalReg(MC_Protocol_REG_t reg, uint32_t value);
int32_t getLocalReg(MC_Protocol_REG_t reg);
void setLocalConfigReg(MC_Protocol_CONFIGREG_t reg, int32_t value);
int32_t getLocalConfigReg(MC_Protocol_CONFIGREG_t reg);

void regIsBeingSet(MC_Protocol_REG_t reg, bool isSet);
void configRegIsBeingSet(MC_Protocol_CONFIGREG_t reg, bool isSet);
bool getFlagAckError( void );
//int32_t getLocalReg(volatile md_reg_t * reg_list, MC_Protocol_REG_t reg);
//void setLocalReg(volatile md_reg_t * reg_list, MC_Protocol_REG_t reg, uint32_t value);

#endif /*__MD_COMM_H*/
