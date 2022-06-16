/**
  ******************************************************************************
  * @file    pedal_speed_sensor.h
  * @author  FTEX inc
  * @brief   This file defines the handles, constantas and function prototypes used in higher level modules for pedal assist
  *
	******************************************************************************
*/


#ifndef __PEDAL_SPEED_SENSOR_H
#define __PEDAL_SPEED_SENSOR_H


#include "pulse_frequency_agt.h"
#include "pulse_frequency_gpt.h"
#include "pedal_torque_sensor.h"

// ================= Structure used to configure a pin ===================== //

typedef struct {
    PedalTorqSensorHandle_t * pTorque;		/* Pointer to torque handle */
    PF_Handle_AGT_t *         pSpulse;      /* Pointer to speed handle */
    
    uint32_t    wPedal_Sensor_Read;
} PAS_Handle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief  Pedal Speed Sensor module Initialization
  @param  PAS_Handle_t handle
  @return None
*/
void PAS_Init(PAS_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor average value calculation
  @param  PAS_Handle_t handle
  @return None
*/
void PAS_CalcTSAvValue(PAS_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor pulse capture calculation
  @param  PAS_Handle_t handle
  @return None
*/
void PAS_CalculateSpeed(PAS_Handle_t* pHandle);
 
/**
  @brief  Pedal Speed Sensor Get Periode value
  @param  PAS_Handle_t handle
  @return Pedal wPedal_Sensor_Read value in useconds
*/
uint32_t PAS_GetPeriodValue(PAS_Handle_t* pHandle);


#endif