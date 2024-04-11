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


//-------------------------------- Defines ----------------------------------- //

//timeout when measuring the time off(not running) of the timer.
#define WHEELSPEED_TIMEOUT_MS          1000

//steps used to increment the variable used to
//measure the time interval where timer is not working.
#define WHEELSPEED_TIME_INCREMENT_MS   5

// ================= Structure used to configure a pin ===================== //
typedef struct {
    
  PulseFrequencyHandle_t * pPulseFrequency;  /* Pointer to speed handle */

  uint8_t    bWheelSpeed_PulsePerRotation;  /* Nunber of pulse per rotation */
  float wWheelSpeed_Read;    /* Wheel Speed Sensor Periode value*/
  uint32_t wWheelSpeedFreq;     /* Wheel Speed sensor frequency calculated value */
  int32_t wWheelSpeedRpm;       /* Wheel Speed sensor rotation per minute calculated value */
  uint16_t wWheelSpeedTimeOut;  /*  variable used to count the maximum time before show time ris not working.  */
    
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
float WheelSpdSensor_GetPeriodValue(WheelSpeedSensorHandle_t* pHandle);

/**
  @brief  Function to return speed in rpm
  @param  WheelSpeedSensorHandle_t handle
  @return Speed in rpm
*/
uint16_t WheelSpdSensor_GetSpeedRPM(WheelSpeedSensorHandle_t* pHandle);


#endif
