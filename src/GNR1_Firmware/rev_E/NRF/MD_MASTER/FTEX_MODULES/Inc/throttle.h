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
	
	int16_t hOffset1;          	 /*< Offset of ADC value vs throttle */
	int8_t bSlope1;									 /*< Gain factor of ADC value vs throttle   */
	uint8_t bDivisor1;								   /*< Scaling factor of ADC value vs throttle   */
	
	int16_t hOffset2;          	 /*< Offset of throttle vs torque */
	int8_t bSlope2;									 /*< Gain factor of throttle vs torque   */
	uint8_t bDivisor2;								   /*< Scaling factor of throttle vs torque   */
	
	int16_t hOffset3;          	 /*< Offset of throttle vs speed */
	int8_t bSlope3;									 /*< Gain factor of throttle vs speed   */
	uint8_t bDivisor3;								   /*< Scaling factor of throttle vs speed   */
	
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
  
	uint16_t hInstADCValue;          /**< It contains latest available insteateonous ADC value.*/
	uint16_t hAvADCValue;          /**< It contains latest available average ADC value.*/
	int16_t hAvThrottleValue;          /**< It contains latest available throttle value.*/
	
	THRO_Param_t hParam;
	
} THRO_Handle_t;


/**
 * @brief Initializes throttle sensing conversions
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 *
 *  @p pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
void THRO_Init( THRO_Handle_t * pHandle );

/**
 * @brief Initializes internal average throttle computed value
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 */
void THRO_Clear( THRO_Handle_t * pHandle );

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
  */
int16_t THRO_CalcAvThrottleValue( THRO_Handle_t * pHandle );

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  * @r AverageThrottle : Current averaged throttle measured (in u16)
  */
int16_t THRO_GetAvThrottleValue( THRO_Handle_t * pHandle );


int16_t THRO_ThrottleToTorque( THRO_Handle_t * pHandle );

int16_t THRO_ThrottleToSpeed( THRO_Handle_t * pHandle );


#endif /*__THROTTLE_H*/

