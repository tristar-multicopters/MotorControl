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

typedef struct
{
	uint16_t hStartValue;
	uint16_t hEndValue;
	int16_t hDefaultMaxTorque;
	
} FLDBK_Handle_t;

/* Function for computing maximum torque based on input value */ 
int16_t FLDBK_CalcTorqueMax(FLDBK_Handle_t * pHandle, uint16_t hValue);

/* Function for applying torque limitation based on input value */
int16_t FLDBK_ApplyTorqueLimitation(FLDBK_Handle_t * pHandle, int16_t hInitialTorque, uint16_t hValue);

#endif
