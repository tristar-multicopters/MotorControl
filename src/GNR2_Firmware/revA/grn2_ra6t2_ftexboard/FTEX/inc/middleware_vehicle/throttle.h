/**
  * @file    throttle.h
  * @brief   This module handles throttle management
  *
  */
    
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __THROTTLE_H
#define __THROTTLE_H

#include "regular_conversion_manager.h"
#include "signal_filtering.h"
#include "delay.h"
#include "ramps.h"

    
#define THROTTLE_SLOPE_FACTOR   100   // Factor used to take a floatign point and make a fraction
                                      // If factor == 100 then 1.25f would make a 125/100 fraction 
#define SAFE_THROTTLE_COUNT_100MS 20 // Called every 5ms, 20*5 = 100ms, which is throttle settle time
#define THROTTLE_ADC_MAX_VALUE 32766
#define THROTTLE_PERCENTAGE_VALUE_FACTOR 10000

/**
  * @brief ThrottleParameters_t structure used for storing throttle user parameters
  */
typedef struct
{              
    uint16_t hOffsetThrottle;           // Offset of ADC value vs throttle
    int16_t  bSlopeThrottle;            // Gain factor of ADC value vs throttle
    int16_t  bDivisorThrottle;          // Scaling factor of ADC value vs throttle   
    uint16_t hMaxThrottle;
    
    uint16_t hOffsetTorque;             // Offset of throttle vs torque 
    int16_t bSlopeTorque;               // Gain factor of throttle vs torque   
    int16_t bDivisorTorque;             // Scaling factor of throttle vs torque   
    
    uint16_t hOffsetSpeed;              // Offset of throttle vs speed 
    float fSlopeSpeed;                  // Gain factor of throttle vs speed   
    uint16_t bDivisorSpeed;             // Scaling factor of throttle vs speed   
    
    float fFilterAlpha;                 // Alpha coefficient for low pass first order butterworth filter
    float fFilterBeta;                  // Beta coefficient for low pass first order butterworth filter
    
    uint16_t hDetectionThreshold;       // Minimum throttle at which throttle is detected
    
    uint16_t ThrottleMaxTorque;
    
    uint16_t DefaultMaxThrottleSpeedKMH;// Maximum KM/H speed that is the default value
    uint16_t MaxThrottleSpeedKMH;       // Maximum KM/H speed that is the current value for the bike 
} ThrottleParameters_t;

/**
  * @brief ThrottleHandle_t structure used for throttle monitoring
  */
typedef struct
{                 
    RegConv_t   Throttle_RegConv;
    uint8_t     bConvHandle;         // Handle to the regular conversion
  
    uint16_t hInstADCValue;          // It contains latest available instantaneous ADC value.
    uint16_t hExtLatestVal;          // Contains the latest external throttle value 
    uint16_t hAvADCValue;            // It contains latest available average ADC value.
    uint16_t hAvThrottleValue;       // It contains latest available throttle value.
    
    bool DisableThrottleOutput;      // Used to prevent the throttle value from requesting power 
                                     // We still read the throttle but simply set the output as 0
    
    bool BlockOffThrottle;           //If this variable is true this means we block throttle
    
    bool extThrottleEnable;          // If this variable is true this means we should ignore adc values and only use throttle injected from a function call
    
    bool SafeStart;                  // Stuck throttle check on start
    
    bool CruiseControlEnable;        // Used to control cruise control 
    int16_t CruiseControlTorqueAvg;  // Used to filter torque value when cruise control is engaged
    
    SignalFilteringHandle_t ThrottleFilter; // Filter structure used to filter out noise.
    
    ThrottleParameters_t hParameters;
    
    Delay_Handle_t * pThrottleStuckDelay;
    
} ThrottleHandle_t;


/**
 * @brief Initializes throttle sensing conversions
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_Init(ThrottleHandle_t * pHandle, Delay_Handle_t * pThrottleStuckDelay);

/**
 * @brief Initializes the rest of throttle sensing conversions after MC initialization
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_Init_Torque(ThrottleHandle_t * pHandle, uint16_t maxTorque);

/**
 * @brief Initializes internal average throttle computed value
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_Clear(ThrottleHandle_t * pHandle);

/**
 * @brief Performs the throttle sensing average computation after an ADC conversion.
 *                  Compute torque value in u16 (0 at minimum throttle and 65535 when max throttle).
 *                  Need to be called periodically.
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_CalcAvThrottleValue(ThrottleHandle_t * pHandle);

/**
 * @brief  Returns latest averaged throttle measured expressed in u16
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval AverageThrottle : Current averaged throttle measured (in u16)
 */
uint16_t Throttle_GetAvThrottleValue(ThrottleHandle_t * pHandle);

/**
 * @brief  Returns latest averaged throttle measured (in % of max throttle value) expressed in u16
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval AverageThrottle : Current averaged throttle measured (in % of max throttle value) (in u16)
 */
uint16_t Throttle_GetAvgPercentageThrottleValue(ThrottleHandle_t *pHandle);

/**
 * @brief  Compute motor torque reference value from current throttle value stored in the handle 
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval torque reference in int16
 */
int16_t Throttle_ThrottleToTorque(ThrottleHandle_t * pHandle);

/**
 * @brief  Compute motor speed reference value from current throttle value stored in the handle 
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval speed reference (todo: unit)
 */
uint16_t Throttle_ThrottleToSpeed(ThrottleHandle_t * pHandle);

/**
 * @brief  Return true if throttled is pressed (threshold is passed) 
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval True if throttle is pressed, false otherwise
 */
bool Throttle_IsThrottleDetected(ThrottleHandle_t * pHandle);

/**
 * @brief  Set the value of the flag to disable throttle output 
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_DisableThrottleOutput(ThrottleHandle_t * pHandle);


/**
 * @brief  Reset the value of the flag to disable throttle output 
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_EnableThrottleOutput(ThrottleHandle_t * pHandle);

/**
 * @brief  Set the max speed in KmH that you can reach with throttle 
 * @param  pHandle : Pointer on Handle of the throttle, aMaxSpeed a maximumu speed
 * @retval void
 */
void Throttle_SetMaxSpeed(ThrottleHandle_t * pHandle, uint16_t aMaxSpeed);

/**
 * @brief  Get the max speed in KmH that you can reach with throttle 
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval current value of the top speed
 */
uint16_t Throttle_GetMaxSpeed(ThrottleHandle_t * pHandle);

/**
 * @brief  Setup the throttle module to accept an external throttle as the input
 * @param  pHandle : Pointer on Handle of the throttle, aMaxValue is the maximum value the throttle can send
 *         aOffset is the value of the throttle when it is in it's resting position 
 * 
 * @retval void
 */
void Throttle_SetupExternal(ThrottleHandle_t * pHandle, uint16_t aMaxValue, uint16_t aOffset);

/**
 * @brief  Used to update the value of the throttle, the source of the external throttle should call this function
 * @param  pHandle : Pointer on Handle of the throttle, aNewVal the newest throttle value.
 * @retval void
 */
void Throttle_UpdateExternal(ThrottleHandle_t * pHandle, uint16_t aNewVal);

/**
 * @brief  Compute slopes for throttle module
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_ComputeSlopes(ThrottleHandle_t * pHandle);

/**
 * @brief  Return the cruise control state
 * @param  pHandle : Pointer on Handle of the throttle,
 * @retval bool state of the cruise control (1:Engaged, 0:Disengaged)
 */ 
bool Throttle_GetCruiseControlState(ThrottleHandle_t * pHandle);
    
/**
 * @brief  Engage the cruise control feature
 * @param  pHandle : Pointer on Handle of the throttle, desired speed for cruise control
 * @retval void
 */
void Throttle_EngageCruiseControl(ThrottleHandle_t * pHandle, uint8_t aSpeed);

/**
 * @brief  Disengage the cruise control feature
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_DisengageCruiseControl(ThrottleHandle_t * pHandle);

/**
 * @brief  Function used to apply a filter on the Torque when engaging Cruise control
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
int16_t Throttle_ApplyCruiseFilter(ThrottleHandle_t * pHandle, int16_t aTorque);

#endif /*__THROTTLE_H*/
