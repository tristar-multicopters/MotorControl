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
void THRO_Init( THRO_Handle_t * pHandle )
{
	/* Need to be register with RegularConvManager */
	pHandle->convHandle = RCM_AddConv(pHandle->pRegularConversionManager, pHandle->hChannelConfig);
	THRO_Clear( pHandle );
}

/**
 * @brief Initializes internal average throttle computed value
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 */
void THRO_Clear( THRO_Handle_t * pHandle )
{
	pHandle->hAvADCValue = 0u;
  pHandle->hAvThrottleValue = 0u;
}

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
  */
int16_t THRO_CalcAvThrottleValue( THRO_Handle_t * pHandle )
{
	uint32_t wtemp;
	int32_t wAux;
  uint16_t hAux1;
	int16_t hAux2;
	uint16_t hBandwidth;
	
	/*
		Compute averaged raw ADC value (between 0 and 65535)
	*/
	hAux1 = RCM_ReadConv(pHandle->pRegularConversionManager, pHandle->convHandle);
	pHandle->hInstADCValue = hAux1;
	
	if (pHandle->hInstADCValue > pHandle->hAvADCValue)
		hBandwidth = pHandle->hParam.hLowPassFilterBW1;
	else
		hBandwidth = pHandle->hParam.hLowPassFilterBW2;

	if ( hAux1 != 0xFFFFu )
	{
		wtemp =  ( uint32_t )( hBandwidth - 1u );
		wtemp *= ( uint32_t ) ( pHandle->hAvADCValue );
		wtemp += hAux1;
		wtemp /= ( uint32_t )( hBandwidth );

		pHandle->hAvADCValue = ( uint16_t ) wtemp;
	}
	
	/*
		Compute throttle value (between -32768 and 32767)
	*/
	wAux = (int32_t)( (pHandle->hAvADCValue/2) - 32767 - pHandle->hParam.hOffset1 );
	if (wAux > INT16_MAX)
		wAux = INT16_MAX;
	else if (wAux < INT16_MIN)
		wAux = INT16_MIN;
	
	hAux2 = (int16_t) wAux;
	
	wAux = (int32_t)(pHandle->hParam.bSlope1 * hAux2);
	wAux /= pHandle->hParam.bDivisor1;
	if (wAux > INT16_MAX)
		wAux = INT16_MAX;
	else if (wAux < INT16_MIN)
		wAux = INT16_MIN;
			
	pHandle->hAvThrottleValue = (int16_t) wAux;
	
	return pHandle->hAvThrottleValue;
}

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  * @r AverageThrottle : Current averaged throttle measured (in u16)
  */
int16_t THRO_GetAvThrottleValue( THRO_Handle_t * pHandle )
{
  return ( pHandle->hAvThrottleValue );
}


int16_t THRO_ThrottleToTorque( THRO_Handle_t * pHandle )
{
	int32_t wAux;
	int16_t hAux;
	
	hAux = pHandle->hAvThrottleValue - pHandle->hParam.hOffset2;
	
	wAux = (int32_t)(pHandle->hParam.bSlope2 * hAux);
	wAux /= pHandle->hParam.bDivisor2;
	if (wAux > INT16_MAX)
		wAux = INT16_MAX;
	else if (wAux < INT16_MIN)
		wAux = INT16_MIN;
			
	return (int16_t)wAux;
}

int16_t THRO_ThrottleToSpeed( THRO_Handle_t * pHandle )
{
}

