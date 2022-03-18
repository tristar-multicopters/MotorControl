/**
  ******************************************************************************
  * @file    pedal_assist.h
  * @author  Ronak Nemade, FTEX
  * @brief   This file defines the handles, constantas and function prototypes used in higher level modules for pedal assist
  *
	******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VC_PEDALASSIST_H
#define __VC_PEDALASSIST_H

#include "stdlib.h"
#include "stdint.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "regular_conversion_manager.h"
#include "board_hardware.h"
#include "mc_defines.h"

// -----------------------------------------This section has defines for types of pedal assist systems-----------------------
typedef enum
{
    PAS_TORQUE,                                /**< @brief Pedal assist has three wires, produces pulses for pedal rotation*/
		PAS_SIN,
		PAS_SINCOS,
} PAS_type_t;


typedef struct {
	
	nrf_drv_timer_t* 	pTimerInstance;
	uint32_t 					pas_timer_timeout_ms;
	
	nrfx_gpiote_pin_t pas_speed_encoder_pin;						// If there is single input speed encoder in terms of pulses

  uint32_t pas_torque_encoder_pin;										// this section has to be configured with rcm module
	
	nrfx_gpiote_pin_t pas_sin_pin;
	nrfx_gpiote_pin_t pas_cos_pin;
	
	PAS_type_t pas_type;
	
} PAS_Config_t;


typedef enum
{
	PAS_LEVEL_0,
	PAS_LEVEL_1,
	PAS_LEVEL_2,
	PAS_LEVEL_3,
	PAS_LEVEL_4,
	PAS_LEVEL_5,
} pas_level_t;


typedef struct {
	
	PAS_Config_t pas_config;
	pas_level_t pas_level;
	bool pas_state;
	
	int16_t hTrefPAS;
	int16_t hIqmax;
	
	pas_level_t hNumberOfLevel;
	uint8_t count;
	
	int16_t hTref;
	uint16_t hLowPassFilterBW;
	
} PAS_Handle_t;


 /**< @brief Function to initialise pedal assist system*/
void PAS_Init(PAS_Handle_t* pHandle);

/**< @brief Function to set the level of pedal assist*/
void PAS_SetLevel(PAS_Handle_t* pHandle, pas_level_t Level);

/**< @brief Function to read motor reference from pedal assist*/
int16_t PAS_CalcIqref(PAS_Handle_t* pHandle,int16_t Tref);



#endif
