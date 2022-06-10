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
#include <stdlib.h>
typedef struct
{
	int16_t hStartValue;
	int16_t hEndValue;
	int16_t hIntervalValue;
	int16_t hDefaultMaxTorque;
	
    uint16_t hSlowStartBandwidth;
    uint32_t wSlowStartTimeout;
    
	bool bEnableFoldback;
	bool bIsInverted;
    bool bEnableSlowStart;
    bool bRefreshSlowStart;
    
	
} FLDBK_Handle_t;

/* Function for computing maximum torque based on input value */ 
int16_t FLDBK_CalcTorqueMax(FLDBK_Handle_t * pHandle, int16_t hValue);

/* Function for applying torque limitation based on input value */
int16_t FLDBK_ApplyTorqueLimitation(FLDBK_Handle_t * pHandle, int16_t hInitialTorque, int16_t hValue);

/* Function for setting the start limitation speed value */
void FLDBK_SetStartValue (FLDBK_Handle_t * pHandle, uint16_t hRefSpeed);

/* Function for setting the end limitation speed value */
void FLDBK_SetEndValue (FLDBK_Handle_t * pHandle, uint16_t hIntervalSpeed);

/* Function for applying a low pass filter to the torque after braking or an OC fault*/
int16_t FLDBK_ApplySlowStart(FLDBK_Handle_t * pHandle, int16_t hTorque);

/* Function used to intiate a slow start */
void FLDBK_EnableSlowStart(FLDBK_Handle_t * pHandle);


#endif
