/**
  ******************************************************************************
  * @file    brake.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles brake sensor management
  *
	******************************************************************************
*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BRAKE_H
#define __BRAKE_H

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

/**
  * @brief Brake_Handle_t structure used for brake sensing
  *
  */
typedef struct
{          
	bool bIsPressed;
	nrfx_gpiote_pin_t wPinNumber;
	bool bIsInvertedLogic;
	
} BRK_Handle_t;


void BRK_Init( BRK_Handle_t * pHandle );
bool BRK_IsPressed( BRK_Handle_t * pHandle );


#endif /*__BRAKE_H*/

