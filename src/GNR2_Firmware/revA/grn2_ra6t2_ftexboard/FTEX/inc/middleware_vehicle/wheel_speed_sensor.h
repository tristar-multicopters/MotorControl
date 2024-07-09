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
#define WHEELSPEED_TIMEOUT_MS          4000

//steps used to increment the variable used to
//measure the time interval where timer is not working.
#define WHEELSPEED_TIME_INCREMENT_MS   5

// ================= Structure used to configure a pin ===================== //
typedef struct {
    PulseFrequencyHandle_t * pPulseFrequency;   /* Pointer to speed handle */
    bool useMotorPulsePerRotation;              /* Indicates whether the wheel speed sensor within the motor is used */
    uint8_t pulsePerRotation;                   /* Number of pulse per rotation */
    float timeOnOneMagnetPercent;               /* Percentage of time that the wheel speed sensor spends on each magnet */
    float periodValue;                          /* Wheel Speed Sensor Periode value*/
    uint32_t frequency;                         /* Wheel Speed sensor frequency calculated value */
    int32_t speedRPM;                           /* Wheel Speed sensor rotation per minute calculated value */
    uint16_t timeout;                           /*  variable used to count the maximum time before show time ris not working.  */ 
} WheelSpeedSensorHandle_t;

// ==================== Public function prototypes ========================= //
/**
  @brief  Function to initialize wheel speed sensor handle
  @param  magnetsPerRotation Number of magnets per rotation on WSS
  @return None
*/
void WSSInit(uint8_t magnetsPerRotation);

/**
  @brief  Function to calculate the wheel speed sensor value
  @param  motorTempSensorMixed flag if bike has a mixed temp/wheel speed sensor
  @return None
*/
void WSSCalculatePeriodValue(bool motorTempSensorMixed);

/**
  @brief  Function to Get periode value in usec
  @return Speed in rpm
*/
float WSSGetPeriodValue(void);

/**
  @brief  Function to return speed in rpm
  @return Speed in rpm
*/
uint16_t WSSGetSpeedRPM(void);

/**
  @brief  Update the pulse capture value coming from the ISR
  @param  Capture : Value capture by the ISR
  @return None
*/
void WSSUpdatePulseFromISR(uint32_t capture);

/**
  @brief  Update the overflow coming from ISR
  @return None
*/
void WSSOverflowPulseFromISR(void);

/**
  @brief  Getter for WSS flag : useMotorPulsePerRotation
  @return Value of useMotorPulsePerRotation
*/
bool WSSGetUseMotorPulsePerRotation(void);

#endif