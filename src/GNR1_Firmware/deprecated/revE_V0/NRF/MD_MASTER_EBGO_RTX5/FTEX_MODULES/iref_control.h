/**
  ******************************************************************************
  * @file    iref_control.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles computation of current reference to motor
  *
	******************************************************************************
	*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IREF_CONTROL_H
#define __IREF_CONTROL_H

#include "mc_defines.h"

typedef struct
{
	int16_t hLowSpeedFluxTorqueRatio;
	int32_t wLowSpeedThreshold;
	int32_t hMaxSqCurrModule;
	
} IREF_Handle_t;


/**
 * @brief Initializes flux reference control module
 *
 *  @p pHandle : Pointer on Handle structure of IREF_Handle_t component
 */
void IREF_Init( IREF_Handle_t * pHandle );

/**
  * @brief Performs the current reference control algorithm
  *  @p pHandle : Pointer on Handle structure of IREF_Handle_t component
  */
qd_t IREF_ComputeIref( IREF_Handle_t * pHandle, int16_t hIqref, int32_t wSpeed );


#endif /*__IREF_CONTROL_H*/

