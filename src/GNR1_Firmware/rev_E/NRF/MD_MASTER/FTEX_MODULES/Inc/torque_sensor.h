/**
  ******************************************************************************
  * @file           : torque_sensor.h
  * @brief          : Header for torque.c file.
  *                   This file contains the functions and methods to use "torque", 
	* @author  				: Jabrane Chakroun
  ******************************************************************************
  * Hardware used:
  *  
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
  *
  * This software is the sole property of FTEX Inc.
  *
  * COPYRIGHT (c) 2022 BY FTEX Inc. ALL RIGHTS RESERVED. NO PART OF
  * THIS PROGRAM OR PUBLICATION MAY BE REPRODUCED, TRANSMITTED, TRANSCRIBED, 
  * STORED IN A RETRIEVAL SYSTEM, OR TRANSLATED INTO ANY LANGUAGE OR COMPUTER 
  * LANGUAGE IN ANY FORM OR BY ANY MEANS, ELECTRONIC, MECHANICAL, MAGNETIC, 
  * OPTICAL, CHEMICAL, MANUAL, OR OTHERWISE, WITHOUT THE PRIOR WRITTEN 
  * PERMISSION OF FTEX Inc.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TORQUE_SENSOR_H
#define __TORQUE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "regular_conversion_manager.h"
#include <math.h>

/* Exported types ------------------------------------------------------------*/

/* Torque_Param_t structure used to store torque user parameters */
typedef struct
{          
  uint16_t 	hLowPassFilterBW1;   /* used to configure the first order software filter bandwidth */
	uint16_t 	hLowPassFilterBW2;	
	
	uint16_t 	hOffset;          	 /* Offset of the torque signal when at lowest position */
	uint16_t 	hMax;             	 /* torque signal when at maximum position */	
	
	int8_t 		m;									 /* Gain factor of torque   */
	uint8_t 	F;								   /* Scaling factor of torque  */
} TS_Param_t;

/* Torque_Handle_t structure used for Torque monitoring */
typedef struct
{          
	nrf_saadc_channel_config_t 	hChannelConfig;					/* It contains analog channel configuration used for torque sensing. */
  uint8_t 										convHandle;            	/* handle to the regular conversion */ 
	RCM_Handle_t* 							pRegularConversionManager;
  
	uint16_t 										hInstTorque;         		/* It contains latest available insteateonous torque.
																												 This parameter is expressed in u16 */
	uint16_t 										hAvTorque;          		/* It contains latest available average torque.
																										 This parameter is expressed in u16 */
	int16_t 										hIqref;            		  /* Current Iqref value */
	TS_Param_t 									hParam;
} TS_Handle_t;



/* Private includes ----------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

// #define all PINS and PORTS here

// #define all HW dependent functions here

void TS_Init( TS_Handle_t * pHandle );
void TS_Clear( TS_Handle_t * pHandle );

uint16_t TS_CalcAvValue( TS_Handle_t * pHandle );
uint16_t TS_GetAvValue( TS_Handle_t * pHandle );

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLE_HW_H */



/****************************************************** (C) COPYRIGHT FTEX Inc *****END OF FILE**** ************************************************/
