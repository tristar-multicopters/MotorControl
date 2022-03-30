/**
  ******************************************************************************
  * @file    throttle.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles throttle management
  *
	******************************************************************************
	*/
	
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __THROTTLE_H
#define __THROTTLE_H

#include "regular_conversion_manager.h"
#include <math.h>

/**
  * @brief Throttle_Param_t structure used for storing throttle user parameters
  *
  */
typedef struct
{          
  uint16_t hLowPassFilterBW1;   /**< used to configure the first order software filter bandwidth.
                                    hLowPassFilterBW = NTC_CalcBusReading
                                    call rate [Hz]/ FilterBandwidth[Hz] */
	uint16_t hLowPassFilterBW2;
	
	uint16_t hOffsetThrottle;          	 /*< Offset of ADC value vs throttle */
	uint8_t bSlopeThrottle;									 /*< Gain factor of ADC value vs throttle   */
	uint8_t bDivisorThrottle;								   /*< Scaling factor of ADC value vs throttle   */
	
	uint16_t hOffsetTorque;          	 /*< Offset of throttle vs torque */
	int8_t bSlopeTorque;									 /*< Gain factor of throttle vs torque   */
	uint8_t bDivisorTorque;								   /*< Scaling factor of throttle vs torque   */
	
	uint16_t hOffsetSpeed;          	 /*< Offset of throttle vs speed */
	int8_t bSlopeSpeed;									 /*< Gain factor of throttle vs speed   */
	uint8_t bDivisorSpeed;								   /*< Scaling factor of throttle vs speed   */
	
} THRO_Param_t;

/**
  * @brief Throttle_Handle_t structure used for throttle monitoring
  *
  */
typedef struct
{          
	nrf_saadc_channel_config_t hChannelConfig;		/**< It contains analog channel configuration used for throttle sensing. */
  uint8_t convHandle;            /*!< handle to the regular conversion */ 
	RCM_Handle_t* pRegularConversionManager;
  
	uint16_t hInstADCValue;          /**< It contains latest available instantaneous ADC value.*/
	uint16_t hAvADCValue;          /**< It contains latest available average ADC value.*/
	uint16_t hAvThrottleValue;          /**< It contains latest available throttle value.*/
	
	THRO_Param_t hParam;
	
} THRO_Handle_t;


/**
 * @brief Initializes throttle sensing conversions
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 *
 *  @p pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
void THRO_Init(THRO_Handle_t * pHandle);

/**
 * @brief Initializes internal average throttle computed value
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 */
void THRO_Clear(THRO_Handle_t * pHandle);

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion.
					 Compute torque value in u16 (0 at minimum throttle and 65535 when max throttle).
					 Need to be called periodically.
  *
  *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
  */
void THRO_CalcAvThrottleValue(THRO_Handle_t * pHandle);

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  * @r AverageThrottle : Current averaged throttle measured (in u16)
  */
uint16_t THRO_GetAvThrottleValue(THRO_Handle_t * pHandle);


int16_t THRO_ThrottleToTorque(THRO_Handle_t * pHandle);

int16_t THRO_ThrottleToSpeed(THRO_Handle_t * pHandle);


#endif /*__THROTTLE_H*/

