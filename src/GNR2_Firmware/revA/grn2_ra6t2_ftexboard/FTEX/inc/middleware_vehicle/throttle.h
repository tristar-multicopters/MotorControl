/**
  * @file    throttle.h
  * @brief   This module handles throttle management
  *
  */
    
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __THROTTLE_H
#define __THROTTLE_H

#include "regular_conversion_manager.h"
#include <math.h>
#include "signal_filtering.h"
#include "foldback.h"
#include "delay.h"
#include "vc_errors_management.h"


#define THROTTLE_STABLE_SPEED_INTERVAL_RPM     70   // Interval of RPM at which we apply the power restriction to stabilize the speed 
#define THROTTLE_STABLE_SPEED_POWER_PERCENT     8   // Amount of power allowed to be used when the stabilisation interval is reached in %
#define THROTTLR_MAJOR_OVER_SPEED_INTERVAL_RPM 22   // How many RPM over max speed is considered Major over speed       

#define THROTTLE_SLOPE_FACTOR   100   // Factor used to take a floatign point and make a fraction
                                      // If factor == 100 then 1.25f would make a 125/100 fraction 
#define SAFE_THROTTLE_COUNT_100MS 20 // Called every 5ms, 20*5 = 100ms, which is throttle settle time

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
    int16_t bSlopeSpeed;                // Gain factor of throttle vs speed   
    uint16_t bDivisorSpeed;             // Scaling factor of throttle vs speed   
    
    float fFilterAlpha;                 // Alpha coefficient for low pass first order butterworth filter
    float fFilterBeta;                  // Beta coefficient for low pass first order butterworth filter
    
    uint16_t hDetectionThreshold;       // Minimum throttle at which throttle is detected
    
    uint16_t MaxSafeThrottleSpeedKMH;   // Maximum KM/H speed that is safe for the bike
    uint16_t DefaultMaxThrottleSpeedKMH;// Maximum KM/H speed that is considered safe 
    
    uint16_t MaxSafeThrottleSpeedRPM;   // Maximum RPM speed that is safe for the bike
    uint16_t MaxThrottleSpeedRPM;// Maximum RPM speed that is considered safe
    
    uint16_t ThrottleDecreasingRange;    
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
    
    bool extThrottleEnable;          // If this variable is true this means we should ignore adc values and only use throttle injected from a function call
    
    bool SafeStart;                  // Stuck throttle check on start
        
    SignalFilteringHandle_t ThrottleFilter; // Filter structure used to filter out noise.
    
    Foldback_Handle_t *SpeedFoldbackVehicleThrottle;    /* Foldback handle for vehicle speed speed */
    
    ThrottleParameters_t hParameters;
    
    Delay_Handle_t * pThrottleStuckDelay;
	
} ThrottleHandle_t;


/**
 * @brief Initializes throttle sensing conversions
 * @param  pHandle : Pointer on Handle of the throttle
 * @retval void
 */
void Throttle_Init(ThrottleHandle_t * pHandle, Delay_Handle_t * pThrottleStuckDelay,  uint32_t MotorToHubGearRatio);

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
 * @brief  Set the max speed in RPM that you can reach with throttle 
 * @param  pHandle : Pointer on Handle of the throttle, aMaxSpeedRPM a maximumu speed in wheel RPM
 * @retval void
 */
void Throttle_SetMaxSpeed(ThrottleHandle_t * pHandle, uint16_t aMaxSpeedRPM);

/**
 * @brief  Setup the throttle module to accept an external throttle as the input
 * @param  pHandle : Pointer on Handle of the throttle, aMaxValue is the maximum value the throttle can send
 *         aOffset is the value of the throttle when it is in it's resting position 
 * 
 * @retval void
 */
void Throttle_SetupExternal(ThrottleHandle_t * pHandle, uint16_t aMaxValue, uint16_t aOffset, uint32_t MotorToHubGearRatio);

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
void Throttle_ComputeSlopes(ThrottleHandle_t * pHandle, uint16_t MotorToHubGearRatio);

#endif /*__THROTTLE_H*/

