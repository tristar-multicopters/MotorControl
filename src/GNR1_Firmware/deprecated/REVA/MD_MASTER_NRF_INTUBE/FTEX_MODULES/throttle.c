/**
  ******************************************************************************
  * @file    throttle.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles throttle management
  *
	******************************************************************************
*/

#include "throttle.h"


/* Functions ---------------------------------------------------- */

/**
 * @brief Initializes throttle sensing conversions
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 *
 *  @p pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
void Throttle_Init( Throttle_Handle_t * pHandle )
{
	/* Need to be register with RegularConvManager */
	pHandle->convHandle = RCM_AddConv(pHandle->hChannelConfig);
	Throttle_Clear( pHandle );
}

/**
 * @brief Initializes internal average throttle computed value
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 */
void Throttle_Clear( Throttle_Handle_t * pHandle )
{
  pHandle->hAvThrottle = 0u;
	pHandle->hIqref = 0;
}

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
  */
void Throttle_CalcAvValue( Throttle_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;

	hAux = RCM_ReadConv(pHandle->convHandle);
	pHandle->hInstThrottle = hAux;

	if ( hAux != 0xFFFFu )
	{
		wtemp =  ( uint32_t )( pHandle->hParam.hLowPassFilterBW ) - 1u;
		wtemp *= ( uint32_t ) ( pHandle->hAvThrottle );
		wtemp += hAux;
		wtemp /= ( uint32_t )( pHandle->hParam.hLowPassFilterBW );

		pHandle->hAvThrottle = ( uint16_t ) wtemp;
	}
}

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  * @r AverageThrottle : Current averaged throttle measured (in u16)
  */
uint16_t Throttle_GetAvValue( Throttle_Handle_t * pHandle )
{
  return ( pHandle->hAvThrottle );
}

/**
  * @brief  Returns Iq computed using latest throttle
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  */
int16_t Throttle_CalcIqref( Throttle_Handle_t * pHandle )
{
	int16_t wIqref = (int16_t)(pHandle->hParam.hSlope*( pHandle->hAvThrottle - pHandle->hParam.hOffset ));
	//pHandle->hIqref = wIqref;
	int16_t lStep = (int16_t) pHandle->hParam.hStep;
	
	if (wIqref >= 0)
	{
		if (wIqref > pHandle->hIqref)
		{
			pHandle->hIqref += lStep;
		}
		else
		{
			pHandle->hIqref = wIqref;
		}
	}
	else
	{
		if (wIqref < pHandle->hIqref)
		{
			pHandle->hIqref -= lStep;
		}
		else
		{
			pHandle->hIqref = wIqref;
		}
	}

	return pHandle->hIqref;
}
