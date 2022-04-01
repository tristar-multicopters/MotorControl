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



#define Coeff_RPM 	60
#define Coeff_FREQ	1000000


/* PAS Level enumeration */
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
	
	uint8_t bMaxLevel;
	uint8_t pRampCoeff;
	
} PAS_Handle_t;


void PAS_Init(PAS_Handle_t* pHandle);

void PAS_CalculateSpeed(PAS_Handle_t* pHandle);

int16_t PAS_GetDirection(PAS_Handle_t* pHandle);
int16_t PAS_GetTorque(PAS_Handle_t* pHandle);
int32_t PAS_GetSpeedValue(PAS_Handle_t* pHandle);


#endif
