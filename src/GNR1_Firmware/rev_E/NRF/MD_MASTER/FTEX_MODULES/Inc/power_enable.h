/**
  ******************************************************************************
  * @file    power_enable.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles power enable pin
  *
	******************************************************************************
*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWER_ENABLE_H
#define __POWER_ENABLE_H

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

typedef struct
{          
	bool bIsPowerEnabled;
	nrfx_gpiote_pin_t wPinNumber;
	bool bIsInvertedLogic;
	bool bIsPowerEnabledPrevious;
	
} PWREN_Handle_t;


void PWREN_Init( PWREN_Handle_t * pHandle );
bool PWREN_IsPowerEnabled( PWREN_Handle_t * pHandle );


#endif /*__POWER_ENABLE_H*/

