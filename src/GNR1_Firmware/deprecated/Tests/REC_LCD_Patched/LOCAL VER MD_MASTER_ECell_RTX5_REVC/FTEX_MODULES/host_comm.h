/**
  ******************************************************************************
  * @file    host_comm.h
  * @author  Jorge Andres Polo, FTEX
	* @author  Sami Bouzid, FTEX
  * @brief   High level module that controls communication between host and nRF52					 
  *
	******************************************************************************
	*/

#include "frame_communication_protocol.h"
#include "storage_management.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HOST_COMM_H
#define __HOST_COMM_H

/* Testing firmware version and MD type*/
#define FW_VERSION				  12
#define IS_DUAL							0

typedef enum{
    SINGLE_M = 1,
    DUAL_M,
}motor_t;
/***************************************/
typedef enum
{
  VC_RT_PROTOCOL_REG_V_FLAGS = 1,
	VC_RT_PROTOCOL_REG_V_STATUS,
	VC_RT_PROTOCOL_REG_V_THROTTLE,
	VC_RT_PROTOCOL_REG_V_BRAKE,
  VC_RT_PROTOCOL_REG_M_FLAGS,
	VC_RT_PROTOCOL_REG_M_STATUS,
  VC_RT_PROTOCOL_REG_M_TORQUE_KP,
  VC_RT_PROTOCOL_REG_M_TORQUE_KI,
  VC_RT_PROTOCOL_REG_M_FLUX_KP,
  VC_RT_PROTOCOL_REG_M_FLUX_KI,
  VC_RT_PROTOCOL_REG_V_BUS_VOLTAGE,
  VC_RT_PROTOCOL_REG_M_HEATS_TEMP,
	VC_RT_PROTOCOL_REG_M_TORQUE_MEAS,
	VC_RT_PROTOCOL_REG_M_FLUX_MEAS,	
  VC_RT_PROTOCOL_REG_M_I_A,
  VC_RT_PROTOCOL_REG_M_I_B,
  VC_RT_PROTOCOL_REG_M_I_Q,
  VC_RT_PROTOCOL_REG_M_I_D,
  VC_RT_PROTOCOL_REG_M_I_Q_REF,
  VC_RT_PROTOCOL_REG_M_I_D_REF,
  VC_RT_PROTOCOL_REG_M_V_Q,
  VC_RT_PROTOCOL_REG_M_V_D,
  VC_RT_PROTOCOL_REG_M_MEAS_ROT_SPEED,
  VC_RT_PROTOCOL_REG_UNDEFINED,
} VC_RT_Protocol_REG_t;

typedef enum
{
  VC_CFG_PROTOCOL_REG_V_WHEELDIA = 26,
	VC_CFG_PROTOCOL_REG_V_MAXSPEED,
	VC_CFG_PROTOCOL_REG_V_GEARRATIO,
	VC_CFG_PROTOCOL_REG_V_THROTTLE_M,
	VC_CFG_PROTOCOL_REG_V_THROTTLE_F,
	VC_CFG_PROTOCOL_REG_V_THROTTLE_OFFSET,
	VC_CFG_PROTOCOL_REG_V_THROTTLE_TYPE,
	VC_CFG_PROTOCOL_REG_V_DUALMOTOR,
	VC_CFG_PROTOCOL_REG_V_MAINMOTOR,
	VC_CFG_PROTOCOL_REG_M_TORQUE_KP,
	VC_CFG_PROTOCOL_REG_M_TORQUE_KI,
	VC_CFG_PROTOCOL_REG_M_FLUX_KP,
	VC_CFG_PROTOCOL_REG_M_FLUX_KI,
	VC_CFG_PROTOCOL_REG_V_DRT_THRESHOLD,
	VC_CFG_PROTOCOL_REG_V_DRT_SLOPE,
	VC_CFG_PROTOCOL_REG_M_ANGLE_OFFSET,
	VC_CFG_PROTOCOL_REG_M_FF_C1,
	VC_CFG_PROTOCOL_REG_M_FF_C2,
	VC_CFG_PROTOCOL_REG_M_FF_C3,
	VC_CFG_PROTOCOL_REG_M_FW_KP,
	VC_CFG_PROTOCOL_REG_M_FW_KI,
	VC_CFG_PROTOCOL_REG_M_MAXVOLT,
	VC_CFG_PROTOCOL_REG_M_MINVOLT,
	VC_CFG_PROTOCOL_REG_M_MAXPHASECURR,
	VC_CFG_PROTOCOL_REG_M_MAXMOTORTEMP,
	VC_CFG_PROTOCOL_REG_M_POLEPAIRS,
  VC_CFG_PROTOCOL_REG_UNDEFINED
} VC_CFG_Protocol_REG_t;

// Frame ID
#define VC_PROTOCOL_CODE_SET_REG          0x01
#define VC_PROTOCOL_CODE_GET_REG          0x02
#define VC_PROTOCOL_CODE_EXECUTE_CMD      0x03
#define VC_PROTOCOL_CODE_GET_FW_VERSION   0x04

// Commands
#define VC_PROTOCOL_CMD_OVERWRITE_FLASH  0x20
#define VC_PROTOCOL_CMD_ENTER_CONFIG     0x21


#define HOST_PENDING_BUFFER_SIZE					4
#define HOSTCOMM_FLAG	0x06

/************* STRUCTURE TO CHANGE.......************/
typedef struct {
	UFCP_Handle_t ufcp_handle;
	uint32_t tx_pin;
	uint32_t rx_pin;
} HOST_UART_Config_t;

typedef struct {
	HOST_UART_Config_t UARTconfig;
	
	VC_Handle_t * pVController;
	uint16_t hStatus;
	uint16_t hError;
	
} HOST_Comm_Handle_t;

/******************************************************/


__NO_RETURN void TSK_HOSTcomm (void * pvParameter);

void HOST_comm_init(HOST_Comm_Handle_t * pHOSTcomm);


#endif
