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
  pHandle->hAvTorque = 0u;
	pHandle->hIqref = 0;
}

/**
	* @brief  Torque ADC value calculation and filtering
	* @param  pHandle used for Torque monitoring
	* @retval None
	*/
uint16_t TS_CalcAvValue( TS_Handle_t * pHandle )
{
  uint32_t ttemp;
  
	uint16_t hBandwidth;

	htorque = RCM_ReadConv(pHandle->pRegularConversionManager, pHandle->convHandle);
	pHandle->hInstTorque = htorque;
	
	if (pHandle->hInstTorque > pHandle->hAvTorque)
		hBandwidth = pHandle->hParam.hLowPassFilterBW1;
	else
		hBandwidth = pHandle->hParam.hLowPassFilterBW2;

	if ( htorque != 0xFFFFu )
	{
		ttemp =  ( uint32_t )( hBandwidth - 1u );
		ttemp *= ( uint32_t ) ( pHandle->hAvTorque );
		ttemp += htorque;
		ttemp /= ( uint32_t )( hBandwidth );

		pHandle->hAvTorque = ( uint16_t ) ttemp;
	}
	return pHandle->hAvTorque;
}

/**
	* @brief  Torque ADC Get value
	* @param  pHandle used for Torque monitoring
	* @retval hAvTorque torque value
	*/
uint16_t TS_GetAvValue( TS_Handle_t * pHandle )
{
  return ( pHandle->hAvTorque );
}
/****************************************************** (C) COPYRIGHT FTEX Inc. *****END OF FILE**** ************************************************/

