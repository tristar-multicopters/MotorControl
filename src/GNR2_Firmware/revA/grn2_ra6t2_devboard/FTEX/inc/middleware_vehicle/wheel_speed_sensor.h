/**
  ******************************************************************************
  * @file    wheel_speed_sensor.h
  * @author  FTEX inc
  * @brief   This file defines the handles, constantas and function prototypes 
  *          used in higher level modules for wheel speed sensor
  *
  ******************************************************************************
*/


#ifndef __WHEEL_SPEED_SENSOR_H
#define __WHEEL_SPEED_SENSOR_H


#include "pulse_frequency_gpt.h"

// =============================== Defines ================================== //

#define WRPMCOEFF       60			// RPM multiplication for r/min
#define WPRECISIONCOEFF 1000	    // ms coefficient precision
#define WCOEFFREQ       1000000000	// Period coeff for usecond division

// ================= Structure used to configure a pin ===================== //

typedef struct {
    WheelFrequency_Handle_GPT_t * wSpulse; /* Pointer to wheel speed handle */
    
    uint32_t wWheel_Sensor_Read; /* Wheel Speed Sensor Periode value*/
    uint32_t wWheelSpeedFreq;   /* Wheel Speed sensor frequency calculated value */
    int32_t wWheelSpeedRpm;     /* Wheel Speed sensor rotation per minute calculated value */
	uint8_t	bWSPulseNumb_pr;    /* Nunber of pulse per rotation */
	
	bool bWSSDetected;		/* Use Wheel speed sensor flag  for detection */
	bool bWSSslowDetect;	/* Use Wheel speed sensor flag  for slow loop detection */
    
	uint8_t bSlowDetectCount; 	/* Wheel speed sensor flag  variable for slow loop detection*/	
	uint8_t bSlowDetectCountValue; /*Wheel speed sensor flag  last variable count for slow loop detection*/	
     
} WheelSpeedSens_Handle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief  Wheel Speed Sensor calculate periode value
  @param  WheelSpeedSens_Handle_t handle
  @return None
*/
void WSS_CalculatePeriodValue(WheelSpeedSens_Handle_t* pHandle);

/**
  @brief  Wheel Speed Sensor Get periode value
  @param  WheelSpeedSens_Handle_t handle
  @return value in uSec
*/
uint32_t WSS_GetPeriodValue(WheelSpeedSens_Handle_t* pHandle);

/**
  @brief  Wheel Speed Sensor Calculate frequency value
  @param  WheelSpeedSens_Handle_t handle
  @return None
*/
void WSS_CalculateSpeedFreq(WheelSpeedSens_Handle_t* pHandle);

/**
  @brief  Wheel Speed Sensor retrun Frequency value
  @param  WheelSpeedSens_Handle_t handle
  @return value in mHz
*/
uint32_t WSS_GetSpeedFreq(WheelSpeedSens_Handle_t* pHandle);

/**
  @brief  Wheel Speed Sensor Calculate RPM value
  @param  WheelSpeedSens_Handle_t handle
  @return None
*/
void WSS_CalculateSpeedRPM(WheelSpeedSens_Handle_t* pHandle);

/**
  @brief  Wheel Speed Sensor Get RPM value
  @param  WheelSpeedSens_Handle_t handle
  @return value in tr/min
*/
int32_t WSS_GetSpeedRPM(WheelSpeedSens_Handle_t* pHandle);

/**
  @brief  Wheel Speed Sensor process Detection
  @param  WheelSpeedSens_Handle_t handle
  @return None
*/
void WSS_UpdateWSSDetection (WheelSpeedSens_Handle_t * pHandle);

/**
  @brief  Wheel Speed Sensor Get Detection
  @param  WheelSpeedSens_Handle_t handle
  @return Boolean
*/
bool WSS_IsWSSDetected(WheelSpeedSens_Handle_t * pHandle);

#endif