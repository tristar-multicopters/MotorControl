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

// =============================== Includes ================================== //
#include "pulse_frequency.h"

// ================= Structure used to configure a pin ===================== //
typedef struct {
    
  PulseFrequencyHandle_t * pPulseFrequency;  /* Pointer to speed handle */

  uint8_t	bPulsePerRotation;  /* Nunber of pulse per rotation */
		
  uint32_t wWheelSpeed_Read;    /* Wheel Speed Sensor Periode value*/
  uint32_t wWheelSpeedFreq;     /* Wheel Speed sensor frequency calculated value */
  int32_t wWheelSpeedRpm;       /* Wheel Speed sensor rotation per minute calculated value */
  
  bool bSpeedDetected;          /* True if speed is detected */
  bool bSpeedslowDetect;        /* Use Wheel speed sensor flag  for slow detection */

  uint8_t bSlowDetectCount;      /* Wheel speed sensor flag  variable for slow loop detection*/	
  uint8_t bSlowDetectCountValue; /*Wheel speed sensor flag  last variable count for slow loop detection*/	
  uint8_t bSpeedslowDetectCorrection; /*Wheel speed /correction factor for slow loop detection on Velec*/
	
} WheelSpeedSensorHandle_t;

// ==================== Public function prototypes ========================= //

/**
  @brief  Function to initialize wheel speed sensor handle
  @param  WheelSpeedSensorHandle_t handle
  @return None
*/
void WheelSpdSensor_Init(WheelSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to calculate the wheel speed sensor value
  @param  WheelSpeedSensorHandle_t handle
  @return None
*/
void WheelSpdSensor_CalculatePeriodValue(WheelSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to Get periode value in usec
  @param  WheelSpeedSensorHandle_t handle
  @return Speed in rpm
*/
uint32_t WheelSpdSensor_GetPeriodValue(WheelSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to  return the wheel speed frequency in mHz
  @param  WheelSpeedSensorHandle_t handle
  @return Speed in rpm
*/
uint32_t WheelSpdSensor_GetSpeedFreq(WheelSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to return speed in rpm
  @param  WheelSpeedSensorHandle_t handle
  @return Speed in rpm
*/
uint16_t WheelSpdSensor_GetSpeedRPM(WheelSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to check if speed is detected
  @param  WheelSpeedSensorHandle_t handle
  @return None
*/
void WheelSpdSensor_UpdateWSSDetection (WheelSpeedSensorHandle_t * pHandle);

/**
  @brief  Function to return the Wheel speed boolean detection
  @param  WheelSpeedSensorHandle_t handle
  @return True if speed is detected
*/
bool WheelSpdSensor_IsSpeedDetected(WheelSpeedSensorHandle_t * pHandle);


#endif
