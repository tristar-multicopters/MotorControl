/**
  ******************************************************************************
  * @file    pedal_assist.h
  * @author  Ronak Nemade, FTEX
  * @brief   This file defines the handles, constantas and function prototypes used in higher level modules for pedal assist
  *
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PEDALASSIST_H
#define __PEDALASSIST_H

#include "stdlib.h"
#include "stdint.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "regular_conversion_manager.h"
#include "speed_pulse_read.h"
#include "torque_sensor.h"


/* Defines -------------------------------------------------------------- */
#define RPMCOEFF 	60			// RPM multiplication for r/min
#define COEFFREQ	1000000	// Period coeff for usecond division

/* PAS Level enumeration ------------------------------------------------ */
typedef enum
{
	PAS_LEVEL_0,
	PAS_LEVEL_1,
	PAS_LEVEL_2,
	PAS_LEVEL_3,
	PAS_LEVEL_4,
	PAS_LEVEL_5,
	PAS_LEVEL_6,
	PAS_LEVEL_7,
	PAS_LEVEL_8,
	PAS_LEVEL_9,
} PAS_sLevel;

typedef struct {
	
	TS_Handle_t * pTorque;		/* Pointer to torque handle */
	
	SPR_Handle_t * pSpulse;		/* Pointer to speed handle */
	
	PAS_sLevel	 	pLevel;					/* Pointer to PAS level enumaration */
	
	uint32_t wPASFreq;
	int32_t wPASRpm;
	
	int16_t bMaxTorque;
	uint8_t bMaxLevel;
		
	uint8_t	bPulseNb;		/* NUMBER_OF_PINS of pulse per rotation */
	
} PAS_Handle_t;

/**
	* @brief  Pedal Assist initialization
	* @param  PAS_Handle_t handle
	* @retval None
	*/
void PAS_Init(PAS_Handle_t* pHandle);

/**
	* @brief  Pedal Assist capture pulse calculation
	* @param  PAS_Handle_t handle
	* @retval None
	*/
void PAS_CalculateSpeed(PAS_Handle_t* pHandle);

/**
	* @brief  Pedal Assist speed Get value
	* @param  PAS_Handle_t handle
	* @retval Pedal sPread value in useconds
	*/
uint32_t PAS_GetPeriodValue(PAS_Handle_t* pHandle);

/**
	* @brief  Pedal Assist speed Get Frequency
	* @param  PAS_Handle_t handle
	* @retval Frequency value in Hz
	*/
uint32_t PAS_GetSpeedFreq(PAS_Handle_t* pHandle);

/**
	* @brief  Pedal Assist speed Get RPM
	* @param  PAS_Handle_t handle
	* @retval Pedal sPAvSpeed value in r/min
	*/
int32_t PAS_GetSpeedRPM(PAS_Handle_t* pHandle);

/**
	* @brief  Pedal Assist capture deirection
	* @param  Forward or back direction
	* @retval Pedal direction in uint8_t
	*/
uint8_t PAS_GetDirection(PAS_Handle_t* pHandle);

/**
	* @brief  Pedal Assist get torque value
	* @param  PAS_Handle_t handle
	* @retval Torque value
	*/
int16_t PAS_GetTorque(PAS_Handle_t* pHandle);



#endif
