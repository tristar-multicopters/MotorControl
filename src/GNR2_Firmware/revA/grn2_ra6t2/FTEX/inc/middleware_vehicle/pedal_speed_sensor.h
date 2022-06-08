/**
  ******************************************************************************
  * @file    pedal_speed_sensor.h
  * @author  Jabrane Chakroun, FTEX
  * @brief   This file defines the handles, constantas and function prototypes used in higher level modules for pedal assist
  *
	******************************************************************************
*/


#ifndef __PEDAL_SPEED_SENSOR_H
#define __PEDAL_SPEED_SENSOR_H


#include "pulse_frequency_agt.h"
#include "pulse_frequency_gpt.h"

// ================= Structure used to configure a pin ===================== //

typedef struct {
	
    PF_Handle_AGT_t * pSpulse;		/* Pointer to speed handle */		
    uint32_t 			wPedal_Sensor_Read;
	
} PAS_Handle_t;

// ==================== Public function prototypes ========================= //

/**
	* @brief  Pedal Assist capture pulse calculation
	* @param  PAS_Handle_t handle
	* @retval None
	*/
void PAS_CalculateSpeed(PAS_Handle_t* pHandle);

/**
	* @brief  Pedal Assist speed Get value
	* @param  PAS_Handle_t handle
	* @retval Pedal wPedal_Sensor_Read value in useconds
	*/
uint32_t PAS_GetPeriodValue(PAS_Handle_t* pHandle);


#endif