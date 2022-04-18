/**
  ******************************************************************************
  * @file    foldback.h
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module manages the vehicle and motor foldbacks 
  *
	******************************************************************************
	*/

#ifndef __FOLDBACK_H
#define __FOLDBACK_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

typedef struct
{
	int16_t hStartValue;
	int16_t hEndValue;
	int16_t hIntervalValue;
	int16_t hDefaultMaxTorque;
	
	bool bEnableFoldback;
	bool bIsInverted;
	
} FLDBK_Handle_t;

/* Function for computing maximum torque based on input value */ 
int16_t FLDBK_CalcTorqueMax(FLDBK_Handle_t * pHandle, int16_t hValue);

/* Function for applying torque limitation based on input value */
int16_t FLDBK_ApplyTorqueLimitation(FLDBK_Handle_t * pHandle, int16_t hInitialTorque, int16_t hValue);

/* Function for setting the start speed limitation speed value */
void FLDBK_SetSpeedStartValue (FLDBK_Handle_t * pHandle, uint16_t hRefSpeed);

/* Function for setting the end speed limitation speed value */
void FLDBK_SetSpeedEndValue (FLDBK_Handle_t * pHandle, uint16_t hIntervalSpeed);

#endif
