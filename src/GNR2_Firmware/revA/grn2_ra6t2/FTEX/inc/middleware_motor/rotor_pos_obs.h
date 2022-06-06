/**
  * @file       rotor_pos_obs.h
  * @brief     	This file provides firmware functions that implement the features
  *             of the Angle Observer component.
  *
*/
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ROTOR_POSITION_OBSERVER_H
#define __ROTOR_POSITION_OBSERVER_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "mc_type.h"
#include "hall_speed_pos_fdbk.h"


/**
  * @brief  This structure is used to handle an instance of the Angle observer component
  */
typedef struct
{ 
	HallPosSensorHandle_t * pHallFdbk;
	
	int16_t hKpGainDef;																		/* Angle Observer default proportional gain */
	int16_t hKpGain;																			/* Angle Observer proportional gain */
	uint16_t hKpDivisorPOW2;															/* Angle Observer proportional divisor by 2 */
	
	int16_t hKiGainDef;																		/* Angle Observer default integral gain */
	int16_t hKiGain;																			/* Angle Observer integral gain */
	uint16_t hKiDivisorPOW2;															/* Angle Observer integral divisor by 2 */
	
	int16_t hKdGainDef;																		/* Angle Observer default derivative gain */
	int16_t hKdGain;																			/* Angle Observer derivative gain */
	uint16_t hKdDivisorPOW2;															/* Angle Observer derivative divisor, power of 2 */

	int16_t hEstMechTorque;																/* Estimated mechanical torque on shaft, half-word */
	int16_t hEstElSpeed;																	/* Estimated speed by the observer, half-word  */
	int16_t hEstElAngle;																	/* Estimated angle by the observer, half-word  */ 
	
	int32_t wEstMechTorque;																/* Estimated mechanical torque on shaft, word */
	int32_t wEstElSpeed;																	/* Estimated speed by the observer, word */
	int32_t wEstElAngle;																	/* Estimated angle by the observer, word */ 
	
} RotorPositionObserverHandle_t;

																																				
// ________________________________Exported functions ------------------------------------------------------- */


/**
  * @brief  It initializes the state observer object
  * @param  pHandle: handler of the current instance of the AO component
  * @retval none
  */
void RotorPosObs_Init( RotorPositionObserverHandle_t * pHandle );

/**
  * @brief  It clears state observer object by re-initializing private variables
  * @param  pHandle: handler of the current instance of the AO component
  * @retval none
  */
void RotorPosObs_Clear( RotorPositionObserverHandle_t * pHandle );

/**
  * @brief  It executes vector tracking state observer to compute new angle estimation
	*					and update the estimated electrical angle. Must be executed at FOC frequency.
  * @param  pHandle: handler of the current instance of the AO component
  * @param  hElTorque: electrical torque currently applied to the shaft (zero if only angle filtering is required)
  * @retval none
  */
int16_t RotorPosObs_CalcElAngle( RotorPositionObserverHandle_t * pHandle, int16_t hElTorque);

/**
  * @brief  It returns the latest estimated electrical angle
  * @param  pHandle: handler of the current instance of the AO component
  * @retval estimated electrical angle
  */
int16_t RotorPosObs_GetElAngle( RotorPositionObserverHandle_t * pHandle);

/**
  * @brief  It returns the latest estimated electrical speed
  * @param  pHandle: handler of the current instance of the AO component
  * @retval estimated electrical speed
  */
int16_t RotorPosObs_GetElSpeed( RotorPositionObserverHandle_t * pHandle);



#endif /*__ROTOR_POSITION_OBSERVER_H*/
