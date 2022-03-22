/**
  ******************************************************************************
  * @file           : torque.h
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
#ifndef __SPEED_PULSE_READ_H
#define __SPEED_PULSE_READ_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "board_hardware.h"
/* Exported types ------------------------------------------------------------*/

typedef enum
{
	Forward,
	Reverse,
	Error_Direction
} SPR_direction_t;

typedef struct
{

	uint16_t 	sLowPassFilterBW1;   /* used to configure the first order software filter bandwidth */
	uint16_t 	sOffset;          	 /* Offset of the torque signal when at lowest position */
	uint16_t 	sMax;             	 /* frequency signal when at maximum position */	
	
} SPR_Param_t;

typedef struct
{
	/* Wheel ppi Capture Channel*/
	uint8_t WCaptureChannel;
	uint8_t WRestartChannel;
	
	
	/* Pedal ppi Capture Channel*/	
	uint8_t bCaptureChannel;
	uint8_t bRestartChannel;
	
	uint16_t bTimer_Prescaler;
	uint32_t bTimer_Width;
	
	/* Pedal Timer */
	nrf_drv_timer_t* 	pTimerInstance;
	/* Wheel Timer */
	nrf_drv_timer_t* 	wTimerInstance;
	
	/* Wheel pulse pin*/
	nrfx_gpiote_pin_t pWheelSpeed_Pulse_pin;
	
	nrfx_gpiote_pin_t pSinSpeed_Pulse_pin;
	nrfx_gpiote_pin_t pCosSpeed_Pulse_pin;

	
	uint16_t 			wPread; 
	uint16_t 			sPread;     
	uint8_t 			Direction_result;	
	
	uint16_t 			sPAvSpeed;       /* It contains latest available pedal average speed in u16 */
	uint16_t 			sWAvSpeed;       /* It contains latest available pedal average speed in u16 */


	SPR_Param_t		sParam;
	
} SPR_Handle_t;



/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void SPR_Init(SPR_Handle_t * pHandle);
void SPWR_Init(SPR_Handle_t * pHandle);

void GPIOTE_Wheel_Capture_Init(SPR_Handle_t* sHandle);

void GPIO_Pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);	
void GPIO_Init(SPR_Handle_t* sHandle);
void GPIOTE_Capture_Init(SPR_Handle_t* sHandle);
void GPIOTE_Wheel_Capture_Init(SPR_Handle_t* sHandle);

uint32_t Pedal_capture_get_vlaue(SPR_Handle_t* sHandle);
uint32_t Wheel_capture_get_vlaue(SPR_Handle_t* sHandle);
uint8_t Get_Drvie_Direction (SPR_Handle_t* sHandle);

uint16_t Pspeed_CalcAvValue( SPR_Handle_t * sHandle );
uint16_t Wspeed_CalcAvValue( SPR_Handle_t * sHandle );
/* Private defines -----------------------------------------------------------*/

// #define all HW dependent functions here


#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLE_HW_H */



/****************************************************** (C) COPYRIGHT FTEX Inc *****END OF FILE**** ************************************************/
