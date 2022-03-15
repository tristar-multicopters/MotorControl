/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __THROTTLE_H
#define __THROTTLE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "regular_conversion_manager.h"

/**
  * @brief Throttle_Handle_t structure used for throttle monitoring
  *
  */
typedef struct
{
  SensorType_t  bSensorType;   /**< Type of instanced throttle sensor.
                                    This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

  RegConv_t      ThrottleRegConv;                                     

  uint16_t hAvThrottle;          /**< It contains latest available average throttle.
                                    This parameter is expressed in u16 */

  uint16_t hExpectedThrottle;    /**< Default set when no sensor available (ie virtual sensor) */
	
  uint16_t hLowPassFilterBW;   /**< used to configure the first order software filter bandwidth.
                                    hLowPassFilterBW = NTC_CalcBusReading
                                    call rate [Hz]/ FilterBandwidth[Hz] */
  uint8_t convHandle;            /*!< handle to the regular conversion */ 
	int16_t hOffset;            /*< Offset of the throttle signal when at lowest position */	
	float hSlope;								/*< Slope of torque vs throttle   */
	int16_t hIqref;            /*< Current Iqref value */
	uint16_t hStep;            /*< Iqref step at each iteration */
	
	uint16_t hDeadzoneLimit;
	
} Throttle_Handle_t;

/* Initialize throttle sensing parameters */
void Throttle_Init( Throttle_Handle_t * pHandle );

/* Clear static average throttle value */
void Throttle_Clear( Throttle_Handle_t * pHandle );

/* Throttle sensing computation */
void Throttle_CalcAvValue( Throttle_Handle_t * pHandle );

/* Get averaged throttle measurement expressed in u16Celsius */
uint16_t Throttle_GetAvValue( Throttle_Handle_t * pHandle );

int16_t Throttle_CalcIqref( Throttle_Handle_t * pHandle );


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __THROTTLE_H */
