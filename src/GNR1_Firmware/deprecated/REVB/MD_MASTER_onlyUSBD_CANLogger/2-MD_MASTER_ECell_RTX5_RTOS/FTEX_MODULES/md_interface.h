/**
  ******************************************************************************
  * @file    vc_config.h
  * @author  Jorge Polo, FTEX
  * @brief   Header of the module that has functions for getting (and setting) the 
  *          values of the motor drive parameters
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MD_INTERFACE_H
#define __MD_INTERFACE_H

#include "stdlib.h"
#include "mc_defines.h"
#include "uart_frame_communication_protocol.h"

#define NB_OF_MOTOR 2

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

//************** PUBLIC FUNCTIONS FOR UPDATING LOCAL REGISTERS VALUES **********//

/* Update control mode */
void MD_setMode(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update motor status */
void MD_setStatus(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update torque Kp gain */
void MD_setTorqueKp(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update torque Ki gain */
void MD_setTorqueKi(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update Flux Kp gain */
void MD_setFluxKp(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update Flux Ki gain  */
void MD_setFluxKi(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update Speed Kp gain  */
void MD_setSpeedKp(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update Speed Ki gain */
void MD_setSpeedKi(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update Bus voltage */
void MD_setBusVoltage(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update Iq measure */
void MD_setIqMeas(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update Id measure */
void MD_setIdMeas(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update motor temperature */
void MD_setMotorTemp(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update speed measure */
void MD_setSpeedMeas(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update speed reference */
void MD_setSpeedRef(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

/* Update flags */
void MD_setFlags(MD_Comm_Handle_t *p_Handle, uint8_t motorselection, int32_t value);

#endif
