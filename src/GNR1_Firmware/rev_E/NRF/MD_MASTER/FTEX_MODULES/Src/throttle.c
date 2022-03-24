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
  pHandle->hAvThrottle = 0u;
}

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
  */
uint16_t THRO_CalcAvValue( THRO_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;
	uint16_t hBandwidth;

	hAux = RCM_ReadConv(pHandle->pRegularConversionManager, pHandle->convHandle);
	pHandle->hInstThrottle = hAux;
	
	if (pHandle->hInstThrottle > pHandle->hAvThrottle)
		hBandwidth = pHandle->hParam.hLowPassFilterBW1;
	else
		hBandwidth = pHandle->hParam.hLowPassFilterBW2;

	if ( hAux != 0xFFFFu )
	{
		wtemp =  ( uint32_t )( hBandwidth - 1u );
		wtemp *= ( uint32_t ) ( pHandle->hAvThrottle );
		wtemp += hAux;
		wtemp /= ( uint32_t )( hBandwidth );

		pHandle->hAvThrottle = ( uint16_t ) wtemp;
	}
	
	return pHandle->hAvThrottle;
}

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  * @r AverageThrottle : Current averaged throttle measured (in u16)
  */
uint16_t THRO_GetAvValue( THRO_Handle_t * pHandle )
{
  return ( pHandle->hAvThrottle );
}


int16_t THRO_ThrottleToTorque( THRO_Handle_t * pHandle )
{
	int32_t wAux;
	uint32_t wAux2;
	uint16_t x;
	
	if(pHandle->hAvThrottle < pHandle->hParam.hOffset)
		x = 0;
	else
	{
		x = pHandle->hAvThrottle - pHandle->hParam.hOffset;
		wAux2 = UINT16_MAX * x;
		wAux2 /= pHandle->hParam.hMax;
		x = (uint16_t) wAux2;
	}
	
	wAux = (int32_t)(pHandle->hParam.m * x);
	wAux /= pHandle->hParam.F;
	if(wAux > INT16_MAX)
		wAux = INT16_MAX;
	else if(wAux < INT16_MIN)
		wAux = INT16_MIN;
			
	return (int16_t)wAux;
}

int16_t THRO_ThrottleToSpeed( THRO_Handle_t * pHandle)
{
	// todo: implementation
	
	// return dummy value so compiler doesn't complain
	return -1;
}

