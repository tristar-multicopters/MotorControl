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


#define TASK_THROTTLE_SAMPLE_TIME_TICK 10


/**
  * @brief Throttle_Param_t structure used for storing throttle user parameters
  *
  */
typedef struct
{          
  uint16_t hLowPassFilterBW;   /**< used to configure the first order software filter bandwidth.
                                    hLowPassFilterBW = NTC_CalcBusReading
                                    call rate [Hz]/ FilterBandwidth[Hz] */
	
	int16_t hOffset;            /*< Offset of the throttle signal when at lowest position */	
	float hSlope;								/*< Slope of torque vs throttle   */
	uint16_t hStep;            /*< Iqref step at each iteration */
	
} Throttle_Param_t;

/**
  * @brief Throttle_Handle_t structure used for throttle monitoring
  *
  */
typedef struct
{          
	nrf_saadc_channel_config_t hChannelConfig;		/**< It contains analog channel configuration used for throttle sensing. */
  uint8_t convHandle;            /*!< handle to the regular conversion */ 
  
	uint16_t hInstThrottle;          /**< It contains latest available insteateonous throttle.
                                    This parameter is expressed in u16 */
	
	uint16_t hAvThrottle;          /**< It contains latest available average throttle.
                                    This parameter is expressed in u16 */
	int16_t hIqref;            /*< Current Iqref value */
	
	Throttle_Param_t hParam;
	
} Throttle_Handle_t;


/**
 * @brief Initializes throttle sensing conversions
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 *
 *  @p pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
void Throttle_Init( Throttle_Handle_t * pHandle );

/**
 * @brief Initializes internal average throttle computed value
 *
 *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
 */
void Throttle_Clear( Throttle_Handle_t * pHandle );

/**
  * @brief Performs the throttle sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of ThrottleSensor component
  */
void Throttle_CalcAvValue( Throttle_Handle_t * pHandle );

/**
  * @brief  Returns latest averaged throttle measured expressed in u16
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  * @r AverageThrottle : Current averaged throttle measured (in u16)
  */
uint16_t Throttle_GetAvValue( Throttle_Handle_t * pHandle );

/**
  * @brief  Returns Iq computed using latest throttle
  *
  * @p pHandle : Pointer on Handle structure of ThrottleSensor component
  *
  */
int16_t Throttle_CalcIqref( Throttle_Handle_t * pHandle );


#endif /*__THROTTLE_H*/

