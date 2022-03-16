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
	bool isPressed;
	nrfx_gpiote_pin_t wPinNumber;
	
} Brake_Handle_t;


void Brake_Init( Brake_Handle_t * pHandle );
bool Brake_IsPressed( Brake_Handle_t * pHandle );
int16_t Brake_CalcIqref( Brake_Handle_t * pHandle, int16_t Iqref );


#endif /*__BRAKE_H*/

