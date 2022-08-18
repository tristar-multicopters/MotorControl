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
	SpdPosFdbkHandle_t Super; /* Base class module */
	
	HallPosSensorHandle_t * pHallSensor;									/* Pointer to hall sensor handle used by rotor position observer */
	
	int16_t hKpGainDef;																		/* Default proportional gain */
	int16_t hKpGain;																			/* Proportional gain */
	uint16_t hKpDivisorPOW2;															/* Proportional divisor by 2 */
	
	int16_t hKiGainDef;																		/* Default integral gain */
	int16_t hKiGain;																			/* Integral gain */
	uint16_t hKiDivisorPOW2;															/* Integral divisor by 2 */
	
	int16_t hKdGainDef;																		/* Default derivative gain */
	int16_t hKdGain;																			/* Derivative gain */
	uint16_t hKdDivisorPOW2;															/* Derivative divisor, power of 2 */

	int16_t hEstMechTorque;																/* Estimated mechanical torque on shaft, half-word */
	int16_t hEstElSpeedDpp;																/* Estimated speed by the observer, half-word  */
	int16_t hEstElAngle;																	/* Estimated angle by the observer, half-word  */ 
	
	int32_t wEstMechTorque;																/* Estimated mechanical torque on shaft, word */
	int32_t wEstElSpeedDpp;																/* Estimated speed by the observer, word */
	int32_t wEstElAngle;																	/* Estimated angle by the observer, word */ 
	
} RotorPositionObserverHandle_t;

																																				
// ________________________________Exported functions ------------------------------------------------------- */


/**
  * @brief  It initializes the state observer object
  * @param  pHandle: handler of the current instance of the RotorPosObs component
  * @retval none
  */
void RotorPosObs_Init(RotorPositionObserverHandle_t * pHandle);

/**
  * @brief  It clears state observer object by re-initializing private variables
  * @param  pHandle: handler of the current instance of the RotorPosObs component
  * @retval none
  */
void RotorPosObs_Clear(RotorPositionObserverHandle_t * pHandle);

/**
  * @brief  It executes vector tracking state observer to compute new angle estimation
	*					and update the estimated electrical angle. Must be executed at FOC frequency.
  * @param  pHandle: handler of the current instance of the AO component
  * @param  hElTorque: electrical torque currently applied to the shaft (zero if only angle filtering is required)
  * @retval none
  */
int16_t RotorPosObs_CalcElAngle(RotorPositionObserverHandle_t * pHandle, int16_t hElTorque);

/**
  * @brief  It convert electrical speed dpp to mechanical speed unit
  * @param  pHandle: handler of the current instance of the RotorPosObs component
  * @retval true = sensor information is reliable
  *         false = sensor information is not reliable
  */
bool RotorPosObs_CalcMecSpeedUnit(RotorPositionObserverHandle_t * pHandle, int16_t * pMecSpeedUnit);

/**
  * @brief  It returns the latest estimated electrical angle
  * @param  pHandle: handler of the current instance of the RotorPosObs component
  * @retval estimated electrical angle
  */
int16_t RotorPosObs_GetElAngle(RotorPositionObserverHandle_t * pHandle);

/**
  * @brief  It returns the latest estimated electrical speed
  * @param  pHandle: handler of the current instance of the RotorPosObs component
  * @retval estimated electrical speed
  */
int16_t RotorPosObs_GetElSpeed(RotorPositionObserverHandle_t * pHandle);



#endif /*__ROTOR_POSITION_OBSERVER_H*/
