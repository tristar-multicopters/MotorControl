/**
  ******************************************************************************
  * @file    derate.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles derating of the device, i.e. when temperature exceed safe area.
  *
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DERATE_H
#define __DERATE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

#define DERATE_MAX_TABLE_SIZE 10

/**
  * @brief Derate_Handle_t structure used for derate management
  *
  */
typedef struct
{
	bool bDeratingON;
	int16_t hTempThreshold;
	int16_t hSlope;
	
	int16_t hTref;									/**< Last computed torque reference value */
	
} DRT_Handle_t;

/* Initialize derating parameters */
void DRT_Init( DRT_Handle_t * pHandle );

/* Compute derated torque */
int16_t DRT_CalcDeratedTorque( DRT_Handle_t * pHandle, int16_t torque, int16_t temperature);


#endif /* __DERATE_H */
