/**
  ******************************************************************************
  * @file       ao_spd_pos_fdbk.h
	*	@author  		Sami Bouzid, FTEX.Inc
  * @author  		Ronak Nemade, FTEX.Inc
  * @brief     	This file provides firmware functions that implement the features
  *               of the Angle Observer  component  for the  Motor Control SDK.
  *
  ******************************************************************************
 **/
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AO_SPDPOSFDBK_H
#define __AO_SPDPOSFDBK_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "mc_type.h"
#include "hall_speed_pos_fdbk.h"


/**
  * @brief  This structure is used to handle an instance of the Angle observer component
  */
typedef struct
{ 
	HALL_Handle_t * pHallFdbk;
	
	int16_t hKpGainDef;
	int16_t hKpGain;																			/* Angle Observer proportional gain */
	uint16_t hKpDivisor;																	/* Angle Observer proportional divisor */
	uint16_t hKpDivisorPOW2;															/* Angle Observer proportional divisor by 2 */
	
	int16_t hKiGainDef;
	int16_t hKiGain;																			/* Angle Observer integral gain */
	uint16_t hKiDivisor;																	/* Angle Observer integral divisor */
	uint16_t hKiDivisorPOW2;															/* Angle Observer integral divisor by 2 */
	
	int16_t hKdGainDef;
	int16_t hKdGain;																			/* Angle Observer derivative gain */
	uint16_t hKdDivisor;																	/* Angle Observer derivative divisor */
	uint16_t hKdDivisorPOW2;															/* Angle Observer integral divisor by 2 */

	int16_t hEstMechTorque;																/* Estimated mechanical torque on shaft */
	int16_t hEstElSpeed;																	/* Estimated speed by the observer */
	int16_t hEstElAngle;																	/* Estimated angle by the observer */ 
	
	int32_t wEstMechTorque;																/* Estimated mechanical torque on shaft */
	int32_t wEstElSpeed;																	/* Estimated speed by the observer */
	int32_t wEstElAngle;																	/* Estimated angle by the observer */ 
	
	int16_t hSpeedFactorGain;
	int16_t hSpeedFactorDiv;
	
} AO_Handle_t;

																																				
// ________________________________Exported functions ------------------------------------------------------- */

/* It initializes the state observer object */
void AO_Init( AO_Handle_t * pHandle );

/* It clears state observer object by re-initializing private variables */
void AO_Clear( AO_Handle_t * pHandle );

/* It executes Vector tracking state observer  to compute  new angle
*  estimation and update the estimated electrical angle. Must be executed at FOC frequency */
int16_t AO_CalcElAngle( AO_Handle_t * pHandle, int16_t hElTorque);

int16_t AO_GetElAngle( AO_Handle_t * pHandle);

int16_t AO_GetElSpeed( AO_Handle_t * pHandle);



#endif /*__AO_SPDPOSFDBK_H*/
