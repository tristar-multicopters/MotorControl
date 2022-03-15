/**
  ******************************************************************************
  * @file    torque_distribution.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles torque distribution between motors
  *
	******************************************************************************
	*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TORQUE_DISTRIBUTION_H
#define __TORQUE_DISTRIBUTION_H

#include "throttle.h"
#include "mc_defines.h"

#define MAX_NUMBER_OF_MOTOR 2

typedef enum
{
	SINGLE_MOTOR,
	DUAL_MOTOR,
} TD_Mode_t;

typedef struct
{          
	TD_Mode_t bMode;
	
	int16_t hDeadzoneValue;
	int16_t hDeadzoneHysteresis;
	bool hDeadzoneFlag;
	uint8_t bMainMotorSelection;
	
	int16_t aTorque[MAX_NUMBER_OF_MOTOR];
	
} TD_Handle_t;


/**
 * @brief Initializes torque distribution module
 *
 *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
 */
void TD_Init( TD_Handle_t * pHandle );

/**
  * @brief Performs the torque distribution algorithm
  *
  *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
  */
void TD_DistributeTorque( TD_Handle_t * pHandle, int16_t hIqref);


int16_t TD_GetTorqueM1(TD_Handle_t * pHandle);
int16_t TD_GetTorqueM2(TD_Handle_t * pHandle);


#endif /*__TORQUE_DISTRIBUTION_H*/

