/**
  ******************************************************************************
  * @file    wheel_speed_sensor.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles wheel speed computation from sensor external sensor
  *
	******************************************************************************
	*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WHEEL_SPEED_SENSOR_H
#define __WHEEL_SPEED_SENSOR_H

/* Includes ----------------------------------------------------------------------*/
#include "stdlib.h"
#include "stdint.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "regular_conversion_manager.h"
#include "speed_pulse_read.h"

/* Defines ----------------------------------------------------------------------*/
#define WRPMCOEFF 			60			// RPM multiplication for r/min
#define WPRECISIONCOEFF	1000	// ms coefficient precision
#define WCOEFFREQ				1000000000	// Period coeff for usecond division

/* Structures ----------------------------------------------------------------------*/
typedef struct {
	
	WPR_Handle_t * wSpulse;				/* Pointer to wheel pulse handle */
	
	uint32_t wWSFreq;
	int64_t wWSRpm;
	
	uint8_t	bWSPulseNb;		/* NUMBER of pulse per wheel rotation */
	
	bool bWSSDetected;		/* USE WSS flag  for detection */
	
	bool bWSSslowDetect;	/* Wheel Speed Pulse detected in a slow Mode */
	uint8_t bSlowDetectCount; 		/* Wheel Speed Pulse detected counter for slow Mode */ 
	uint8_t bSlowDetectCountValue;/* Wheel Speed Pulse detected counter value for slow Mode */ 
	
} WSS_Handle_t;


/* Public Functions ----------------------------------------------------------------------*/
/**
	* @brief  Wheel Speed Sensor initialization
	* @param  WSS_Handle_t handle
	* @retval None
	*/
void WSS_Init(WSS_Handle_t* pHandle);

/**
	* @brief  Wheel Speed Sensor calculation
	* @param  WSS_Handle_t handle
	* @retval None
	*/
void WSS_CalculateSpeed(WSS_Handle_t* pHandle);

/**
	* @brief  Wheel Speed Sensor Get value
	* @param  WSS_Handle_t handle
	* @retval Wheel Pulse wPread value in useconds
	*/
int32_t WSS_GetPeriodValue(WSS_Handle_t* pHandle);

/**
	* @brief  Wheel speed sensor Get Frequency
	* @param  WSS_Handle_t handle
	* @retval Frequency value in mHz
	*/
uint32_t WSS_GetSpeedFreq(WSS_Handle_t* pHandle);

/**
	* @brief  Wheel speed senor Calculate RPM
	* @param  WSS_Handle_t handle
	* @retval None
	*/ 
void WSS_CalculateSpeedRPM(WSS_Handle_t* pHandle);

/**
	* @brief  Wheel speed senor Get RPM
	* @param  WSS_Handle_t handle
	* @retval Wheel Speed wWSRpm value in r/min
	*/
int32_t WSS_GetSpeedRPM(WSS_Handle_t* pHandle);

/**
	* @brief  Check the WSS Presence Flag
	* @param  WSS_Handle_t handle
	* @retval 
	*/
void WSS_UpdateWSSDetection (WSS_Handle_t * pHandle);

/**
	* @brief  Return if the wheel speed sensor is moving or not
	* @param  WSS_Handle_t handle
	* @retval True if wheel movement is detected, false otherwise
	*/
bool WSS_IsWSSDetected(WSS_Handle_t * pHandle) ;

#endif /*__WHEEL_SPEED_SENSOR_H*/

