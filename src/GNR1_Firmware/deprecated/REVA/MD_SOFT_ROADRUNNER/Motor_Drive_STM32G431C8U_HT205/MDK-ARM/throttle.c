/* Includes ------------------------------------------------------------------*/
#include "throttle.h"


/* Functions ---------------------------------------------------- */

/**
 * @brief Initializes throttle sensing conversions
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 *
 *  @p pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
__weak void Throttle_Init( Throttle_Handle_t * pHandle )
{

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    /* Need to be register with RegularConvManager */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->ThrottleRegConv);
    Throttle_Clear( pHandle );
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hAvThrottle = pHandle->hExpectedThrottle;
  }

}

/**
 * @brief Initializes internal average throttle computed value
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 */
__weak void Throttle_Clear( Throttle_Handle_t * pHandle )
{
  pHandle->hAvThrottle = 0u;
	pHandle->hIqref = 0;
}

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
  */
__weak void Throttle_CalcAvValue( Throttle_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if ( hAux != 0xFFFFu )
    {
      wtemp =  ( uint32_t )( pHandle->hLowPassFilterBW ) - 1u;
      wtemp *= ( uint32_t ) ( pHandle->hAvThrottle );
      wtemp += hAux;
      wtemp /= ( uint32_t )( pHandle->hLowPassFilterBW );

      pHandle->hAvThrottle = ( uint16_t ) wtemp;
    }
  }
  else  /* case VIRTUAL_SENSOR */
  {
  }
}

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  * @r AverageThrottle : Current averaged throttle measured (in u16)
  */
__weak uint16_t Throttle_GetAvValue( Throttle_Handle_t * pHandle )
{
  return ( pHandle->hAvThrottle );
}

/**
  * @brief  Returns Iq computed using latest throttle
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  */
__weak int16_t Throttle_CalcIqref( Throttle_Handle_t * pHandle )
{
	int16_t wIqref = (int16_t)(pHandle->hSlope*( pHandle->hAvThrottle - pHandle->hOffset ));
	//pHandle->hIqref = wIqref;
	int16_t lStep = (int16_t) pHandle->hStep;
	
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
