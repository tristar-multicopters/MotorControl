/**
  * @file    throttle.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles throttle management
  *
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __THROTTLE_H
#define __THROTTLE_H

#include "regular_conversion_manager.h"
#include <math.h>

/**
  * @brief Throttle_Param_t structure used for storing throttle user parameters
  */
typedef struct
{          
  uint16_t hLowPassFilterBW1;   // used to configure the first order software filter bandwidth.
                                // hLowPassFilterBw = NTC_CalcBusReading
                                // call rate [Hz]/ FilterBandwidth[Hz] 
	uint16_t hLowPassFilterBW2;
	
	uint16_t hOffsetThrottle;            // Offset of ADC value vs throttle
	uint16_t bSlopeThrottle;              // Gain factor of ADC value vs throttle
	uint16_t bDivisorThrottle;            // Scaling factor of ADC value vs throttle   
	
	uint16_t hOffsetTorque;          	 // Offset of throttle vs torque 
	int16_t bSlopeTorque;                 // Gain factor of throttle vs torque   
	uint16_t bDivisorTorque;              // Scaling factor of throttle vs torque   
	
	uint16_t hOffsetSpeed;               // Offset of throttle vs speed 
	int16_t bSlopeSpeed;                  // Gain factor of throttle vs speed   
	uint16_t bDivisorSpeed;               // Scaling factor of throttle vs speed   
	
	uint16_t hDetectionThreshold;        // Minimum throttle at which throttle is detected
	
} THRO_Param_t;

/**
  * @brief Throttle_Handle_t structure used for throttle monitoring
  */
typedef struct
{                 
	RegConv_t   Throttle_RegConv;
    uint8_t     bConvHandle;         // Handle to the regular conversion
  
	uint16_t hInstADCValue;          // It contains latest available instantaneous ADC value.
	uint16_t hAvADCValue;            // It contains latest available average ADC value.
	uint16_t hAvThrottleValue;       // It contains latest available throttle value.
	
	THRO_Param_t hParameters;
	
} THRO_Handle_t;


/**
 * @brief Initializes throttle sensing conversions
 * @param pHandle : Pointer on Handle structure of ThrottleSensor component
 * @param pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
void THRO_Init(THRO_Handle_t * pHandle);

/**
 * @brief Initializes internal average throttle computed value
 * @param pHandle : Pointer on Handle structure of ThrottleSensor component
 */
void THRO_Clear(THRO_Handle_t * pHandle);

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion.
					 Compute torque value in u16 (0 at minimum throttle and 65535 when max throttle).
					 Need to be called periodically.
  * @param pHandle : Pointer on Handle structure of ThrottleSensor component
  */
void THRO_CalcAvThrottleValue(THRO_Handle_t * pHandle);

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  * @param pHandle : Pointer on Handle structure of ThrottleSensor component
  * @retval AverageThrottle : Current averaged throttle measured (in u16)
  */
uint16_t THRO_GetAvThrottleValue(THRO_Handle_t * pHandle);

/**
  * @brief  Compute motor torque reference value from current throttle value stored in the handle 
  * @param pHandle : Pointer on Handle structure of ThrottleSensor component
  * @retval torque reference in int16
  */
int16_t THRO_ThrottleToTorque(THRO_Handle_t * pHandle);

/**
  * @brief  Compute motor speed reference value from current throttle value stored in the handle 
  * @param pHandle : Pointer on Handle structure of ThrottleSensor component
  * @retval speed reference (todo: unit)
  */
int16_t THRO_ThrottleToSpeed(THRO_Handle_t * pHandle);

/**
	* @brief  Return true if throttled is pressed (threshold is passed) 
	* @param  pHandle : Pointer on Handle structure of ThrottleSensor component
	* @retval True if throttle is pressed, false otherwise
	*/
bool THRO_IsThrottleDetected(THRO_Handle_t * pHandle);


#endif /*__THROTTLE_H*/

