/**
  * @file    wheel.h
  * @brief   the vehicle-control wheel parameters and utilities management
  */
  
#ifndef __WHEEL_H
#define __WHEEL_H

#include "stdint.h"
#include "wheel_speed_sensor.h"

#ifdef DEFAULT_WHEEL_DIAMETER_OVERRIDE
    #define WHEEL_DIAMETER  DEFAULT_WHEEL_DIAMETER_OVERRIDE
#else
    #define WHEEL_DIAMETER  WHEEL_DIAMETER_DEFAULT
#endif

/**
  * @brief  Initialize the vc wheel module
  * @param  Wheel diameter
  */
void Wheel_Init(void);

/**
  * @brief  Return the wheel diameter
  * @retval Wheel diameter in inches
  */
uint8_t Wheel_GetWheelDiameter(void);

/**
  * @brief  Set the wheel diameter
  * @param  Wheel diameter in inches
  */
void Wheel_SetWheelDiameter(uint8_t diameterInInches);

/**
  * @brief  Set the wheel diameter to a value in inches by external factor 
  *         we override the flag to false
  */
void Wheel_ExternalSetWheelDiameter(uint8_t diameterInInches);

/**
  * @brief  Check if we had an internal change of the wheel diameter
  */
bool Wheel_CheckInternalUpdateFlag(void);

/**
  * @brief  Clear the internal update flag after processing the change in CAN 
  */
void Wheel_ClearInternalUpdateFlag(void);

/**
  * @brief  Get the speed from the wheel rpm
  * @param  Wheel rpm
  * @retval Speed in km/h
  */
uint16_t Wheel_GetSpeedFromWheelRpm(uint16_t wheelRpm);

/**
  * @brief  Get the wheel rpm from the speed
  * @param  Speed in km/h
  * @retval Wheel rpm
  */
uint16_t Wheel_GetWheelRpmFromSpeed(uint16_t speed);

/**
  * @brief  Get the vehicle speed in kmh using the wheel speed sensor
  * @param  Handle of the wheel speed sensor
  * @retval Speed in km/h
  */
uint16_t Wheel_GetVehicleSpeedFromWSS(WheelSpeedSensorHandle_t * pHandle);

#endif