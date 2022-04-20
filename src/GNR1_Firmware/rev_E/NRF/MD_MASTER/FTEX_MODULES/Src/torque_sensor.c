/**
  ******************************************************************************
  * @file           : torque.c
  * @brief          : This module handles torque general read functions based on the
											regular_conversion_manager.c task
	* @author  				: Jabrane Chakroun
  ******************************************************************************
  * Hardware used:
  *  	P0.02/AIN0: NRF_SAADC_INPUT_AIN0: Torque_Pin 
	*			=> Defined in vc_config.c
  *  
  * ------------------------------
  * Based on software:
  *  	
  *  
  * ------------------------------
  * Version number:
  *  0.0.1
  * ------------------------------
  * Last modified:
  *  07.03.2022 
  * by 
  *  Jabrane Chakroun
  * ------------------------------
  *
  * This software is the sole property of FTEX Inc.
  *
  * COPYRIGHT (c) 2022 BY FTEX Inc. ALL RIGHTS RESERVED. NO PART OF
  * THIS PROGRAM OR PUBLICATION MAY BE REPRODUCED, TRANSMITTED, TRANSCRIBED, 
  * STORED IN A RETRIEVAL SYSTEM, OR TRANSLATED INTO ANY LANGUAGE OR COMPUTER 
  * LANGUAGE IN ANY FORM OR BY ANY MEANS, ELECTRONIC, MECHANICAL, MAGNETIC, 
  * OPTICAL, CHEMICAL, MANUAL, OR OTHERWISE, WITHOUT THE PRIOR WRITTEN 
  * PERMISSION OF FTEX Inc.
  *
  ******************************************************************************
  */
	
/******************************************************************************* Includes ********************************************************************************/

#include "torque_sensor.h"

/**************************************************************************** Private definitions ************************************************************************/
uint16_t htorque;
/****************************************************************** Public Hardware dependent functions ******************************************************************/

/**
	* @brief  Torque ADC hardware initialization
	* @param  pHandle used for Torque monitoring
	* @retval None
	*/
void TS_Init( TS_Handle_t * pHandle )
{
	/* Need to be register with RegularConvManager */
	pHandle->convHandle = RCM_AddConv(pHandle->pRegularConversionManager, pHandle->hChannelConfig);
	TS_Clear( pHandle );
}

/**
	* @brief  Torque ADC hardware values clear
	* @param  pHandle used for Torque monitoring
	* @retval None
	*/
void TS_Clear( TS_Handle_t * pHandle )
{
  pHandle->hAvTorqueValue = 0u;
	pHandle->hAvADCValue = 0u;
}

/**
	* @brief  Torque ADC value calculation and filtering
	* @param  pHandle used for Torque monitoring
	* @retval hAvTorqueValue un uint16_t
	*/
void TS_CalcAvValue( TS_Handle_t * pHandle )
{
  uint32_t tTorqaux;
  uint16_t hTorqux;
	
	
	uint16_t hBandwidth;

	htorque = RCM_ReadConv(pHandle->pRegularConversionManager, pHandle->convHandle);
	pHandle->hInstTorque = htorque;
	
	if (pHandle->hInstTorque > pHandle->hAvADCValue)
		hBandwidth = pHandle->hParam.hLowPassFilterBW1;
	else
		hBandwidth = pHandle->hParam.hLowPassFilterBW2;

	if ( htorque != 0xFFFFu )
	{
		tTorqaux =  ( uint32_t )( hBandwidth - 1u );
		tTorqaux *= ( uint32_t ) ( pHandle->hAvADCValue );
		tTorqaux += htorque;
		tTorqaux /= ( uint32_t )( hBandwidth );

		pHandle->hAvADCValue = ( uint16_t ) tTorqaux;
	}

	/* Compute torque sesnor value (between 0 and 65535) */
	hTorqux = (pHandle->hAvADCValue > pHandle->hParam.hOffsetTS) ? 
					(pHandle->hAvADCValue - pHandle->hParam.hOffsetTS) : 0; //Substraction without overflow
	
	tTorqaux = (uint32_t)(pHandle->hParam.bSlopeTS * hTorqux);
	tTorqaux /= pHandle->hParam.bDivisorTS;
	if (tTorqaux > UINT16_MAX)
		tTorqaux = UINT16_MAX;
	hTorqux = (uint16_t)tTorqaux;
	
	pHandle->hAvTorqueValue = hTorqux;	
}

/**
	* @brief  Torque ADC Get value
	* @param  pHandle used for Torque monitoring
	* @retval hAvTorque torque value
	*/
uint16_t TS_GetAvValue( TS_Handle_t * pHandle )
{
  return ( pHandle->hAvTorqueValue );
}

/**
	* @brief  Convert Torque sensor data to motor torque
	* @param  pHandle used for Torque monitoring
	* @retval hAvTorque torque value
	*/
int16_t TS_ToMotorTorque(TS_Handle_t * pHandle)
{
	int32_t tAux;
	
	/* Compute torque value (between -32768 and 32767) */
	tAux = (int32_t)(pHandle->hAvTorqueValue - pHandle->hParam.hOffsetMT);
	if (tAux < 0)
		tAux = 0;
	/* Use slope factor */
	tAux = ((pHandle->hParam.bSlopeMT)*tAux);
	tAux /= pHandle->hParam.bDivisorMT;
	
	/* Data limitation secure */
	if (tAux > INT16_MAX)
		tAux = INT16_MAX;
	else if (tAux < INT16_MIN)
		tAux = INT16_MIN;
	
	return (int16_t)tAux;
}
/****************************************************** (C) COPYRIGHT FTEX Inc. *****END OF FILE**** ************************************************/

