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
#include "signal_filtering.h"

/**
  * @brief ThrottleParameters_t structure used for storing throttle user parameters
  */
typedef struct
{          	
	uint16_t hOffsetThrottle;           // Offset of ADC value vs throttle
	uint16_t bSlopeThrottle;            // Gain factor of ADC value vs throttle
	uint16_t bDivisorThrottle;          // Scaling factor of ADC value vs throttle   
	
	uint16_t hOffsetTorque;          	// Offset of throttle vs torque 
	int16_t bSlopeTorque;               // Gain factor of throttle vs torque   
	uint16_t bDivisorTorque;            // Scaling factor of throttle vs torque   
	
	uint16_t hOffsetSpeed;              // Offset of throttle vs speed 
	int16_t bSlopeSpeed;                // Gain factor of throttle vs speed   
	uint16_t bDivisorSpeed;             // Scaling factor of throttle vs speed   
    
    float fFilterAlpha;                 // Alpha coefficient for low pass first order butterworth filter
    float fFilterBeta;                  // Beta coefficient for low pass first order butterworth filter
	
	uint16_t hDetectionThreshold;       // Minimum throttle at which throttle is detected
	
} ThrottleParameters_t;

/**
  * @brief ThrottleHandle_t structure used for throttle monitoring
  */
typedef struct
{                 
	RegConv_t   Throttle_RegConv;
    uint8_t     bConvHandle;         // Handle to the regular conversion
  
	uint16_t hInstADCValue;          // It contains latest available instantaneous ADC value.
	uint16_t hAvADCValue;            // It contains latest available average ADC value.
	uint16_t hAvThrottleValue;       // It contains latest available throttle value.
    
    bool DisableThrottleOutput;      // Used to prevent the throttle value from requesting power 
                                     // We still read the throttle but simply set the output as 0
    
    SignalFilteringHandle_t ThrottleFilter; // Filter structure used to filter out noise.
	
	ThrottleParameters_t hParameters;
	
} ThrottleHandle_t;


/**
 * @brief Initializes throttle sensing conversions
 * @param pHandle : Pointer on Handle structure of ThrottleSensor component
 * @param pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
void Throttle_Init(ThrottleHandle_t * pHandle);

/**
 * @brief Initializes internal average throttle computed value
 * @param pHandle : Pointer on Handle structure of ThrottleSensor component
 */
void Throttle_Clear(ThrottleHandle_t * pHandle);

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion.
					 Compute torque value in u16 (0 at minimum throttle and 65535 when max throttle).
					 Need to be called periodically.
  * @param pHandle : Pointer on Handle structure of ThrottleSensor component
  */
void Throttle_CalcAvThrottleValue(ThrottleHandle_t * pHandle);

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  * @param pHandle : Pointer on Handle structure of ThrottleSensor component
  * @retval AverageThrottle : Current averaged throttle measured (in u16)
  */
uint16_t Throttle_GetAvThrottleValue(ThrottleHandle_t * pHandle);

/**
  * @brief  Compute motor torque reference value from current throttle value stored in the handle 
  * @param pHandle : Pointer on Handle structure of ThrottleSensor component
  * @retval torque reference in int16
  */
int16_t Throttle_ThrottleToTorque(ThrottleHandle_t * pHandle);

/**
  * @brief  Compute motor speed reference value from current throttle value stored in the handle 
  * @param pHandle : Pointer on Handle structure of ThrottleSensor component
  * @retval speed reference (todo: unit)
  */
int16_t Throttle_ThrottleToSpeed(ThrottleHandle_t * pHandle);

/**
	* @brief  Return true if throttled is pressed (threshold is passed) 
	* @param  pHandle : Pointer on Handle structure of ThrottleSensor component
	* @retval True if throttle is pressed, false otherwise
	*/
bool Throttle_IsThrottleDetected(ThrottleHandle_t * pHandle);

/**
	* @brief  Set the value of the flag to disable throttle output 
	* @param  pHandle : Pointer on Handle structure of ThrottleSensor component
	* @retval void
	*/
void Throttle_DisableThrottleOutput(ThrottleHandle_t * pHandle);


/**
	* @brief  Reset the value of the flag to disable throttle output 
	* @param  pHandle : Pointer on Handle structure of ThrottleSensor component
	* @retval void
	*/
void Throttle_EnableThrottleOutput(ThrottleHandle_t * pHandle);

#endif /*__THROTTLE_H*/

