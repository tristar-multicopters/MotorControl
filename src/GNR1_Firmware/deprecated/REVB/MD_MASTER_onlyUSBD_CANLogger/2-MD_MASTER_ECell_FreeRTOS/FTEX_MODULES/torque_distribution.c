/**
  ******************************************************************************
  * @file    torque_distribution.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles torque distribution between motors
  *
	******************************************************************************
*/

#include "torque_distribution.h"


/* Functions ---------------------------------------------------- */

/**
 * @brief Initializes torque distribution module
 *
 *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
 */
void TD_Init( TD_Handle_t * pHandle )
{
}

/**
  * @brief Performs the torque distribution algorithm
  *
  *  @p pHandle : Pointer on Handle structure of TD_Handle_t component
  */
void TD_DistributeTorque( TD_Handle_t * pHandle, int16_t hIqref)
{
	if(pHandle->bMode == SINGLE_MOTOR)
	{
		pHandle->aTorque[M1] = hIqref;
		pHandle->aTorque[M2] = 0;
	}
	if(pHandle->bMode == DUAL_MOTOR)
	{
		pHandle->aTorque[M1] = hIqref;
		pHandle->aTorque[M2] = hIqref;
	}
}

int16_t TD_GetTorqueM1(TD_Handle_t * pHandle)
{
	return pHandle->aTorque[M1];
}

int16_t TD_GetTorqueM2(TD_Handle_t * pHandle)
{
	return pHandle->aTorque[M2];
}

