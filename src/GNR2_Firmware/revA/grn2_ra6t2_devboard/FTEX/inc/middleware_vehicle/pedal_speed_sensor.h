/**
  ******************************************************************************
  * @file    pedal_speed_sensor.h
  * @author  FTEX inc
  * @brief   This file defines the handles, constantas and function prototypes 
  *           used in higher level modules for pedal speed sensor
  *
	******************************************************************************
*/


#ifndef __PEDAL_SPEED_SENSOR_H
#define __PEDAL_SPEED_SENSOR_H


#include "pulse_frequency_agt.h"
#include "pedal_torque_sensor.h"

// =============================== Defines ================================== //

#define RPMCOEFF        60      // RPM multiplication for r/min
#define PRECISIONCOEFF	1000	// ms coefficient precision
#define COEFFREQ        1000000000	// Period coeff for usecond division

// ================= Structure used to configure a pin ===================== //

typedef struct {
    PedalTorqSensorHandle_t *       pTorque;    /* Pointer to torque handle */
    PulseFrequency_Handle_AGT_t *   pSpulse;    /* Pointer to speed handle */
    
    
    uint32_t wPedalSpeedSens_Freq;  /* Pedal Speed sensor frequency calculated value */
    uint32_t wPedal_Sensor_Read;    /* Pedal Speed Sensor Periode value*/
    int32_t  wPedalSpeedSens_RPM;   /* Pedal Speed sensor RPM calculated value */
    
    uint8_t	bPulseNumb_pr;   /* Nunber of pulse per rotation */
    
    bool bPSSDetected;      /* Use PSS flag  for detection */
    bool bTorqueSensorUsed; /* Torque sensor use flag */
    
} PedalSpeedSens_Handle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief  Pedal Speed Sensor module Initialization
  @param  PedalSpeedSens_Handle_t handle
  @return None
*/
void PSS_Init(PedalSpeedSens_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor average value calculation
  @param  PedalSpeedSens_Handle_t handle
  @return None
*/
void PSS_CalcTSAvValue(PedalSpeedSens_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor pulse capture calculation
  @param  PedalSpeedSens_Handle_t handle
  @return None
*/
void PSS_CalculateSpeed(PedalSpeedSens_Handle_t* pHandle);
 
/**
  @brief  Pedal Speed Sensor Get Periode value
  @param  PedalSpeedSens_Handle_t handle
  @return Pedal wPedal_Sensor_Read value in useconds
*/
uint32_t PSS_GetPeriodValue(PedalSpeedSens_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor Calculate Frequency value
  @param  PedalSpeedSens_Handle_t handle
  @retval None
*/
void PSS_CalculateSpeedFreq(PedalSpeedSens_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor Get Frequency value
  @param  PedalSpeedSens_Handle_t handle
  @retval Frequency value in mHz
*/
uint32_t PSS_GetSpeedFreq(PedalSpeedSens_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor Calcualte RPM value
  @param  PedalSpeedSens_Handle_t handle
  @retval None
*/
void PSS_CalculateSpeedRPM(PedalSpeedSens_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor Get RPM value
  @param  PedalSpeedSens_Handle_t handle
  @retval Pedal sPAvSpeed value in r/min
*/
int32_t PSS_GetSpeedRPM(PedalSpeedSens_Handle_t* pHandle);

/**
  @brief  Pedal Speed Sensor Presence Flag 
  @param  PedalSpeedSens_Handle_t handle
  @retval None
*/
void PSS_UpdatePASDetection (PedalSpeedSens_Handle_t * pHandle);

/**
  @brief  Pedal Speed Sensor Flag Detection 
  @param  PedalSpeedSens_Handle_t handle
  @retval True if pedal movement is detected, false otherwise
*/
bool PSS_IsPASDetected(PedalSpeedSens_Handle_t * pHandle);



#endif