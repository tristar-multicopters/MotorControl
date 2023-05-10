/**
  * @file    ntc_temperature_sensor.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          Temperature Sensor component of the Motor Control application.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NTC_TEMPERATURESENSOR_H
#define __NTC_TEMPERATURESENSOR_H

#include "mc_type.h"
#include "regular_conversion_manager.h"

#include "lookup_table.h"

extern const uint8_t MIMIMUM_NTC_FREEZING;

typedef struct
{
  SensorType_t  bSensorType;   /**< Type of instanced temperature.
                                    This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

  RegConv_t      TempRegConv;

  uint16_t hAvTempDigital;          /**< It contains latest available average temperature.
                                    This parameter is expressed in u16Celsius */
  int16_t hAvTempCelcius;            /**< It contains latest computation of temperature in Celcius */

  uint16_t hExpectedTempDigital;    /**< Default set when no sensor available (ie virtual sensor) */

  uint16_t hExpectedTempCelcius;    /**< Default value when no sensor available (ie virtual sensor).
                                    This parameter is expressed in Celsius */

  uint16_t hFaultState;        /**< Contains latest Fault code.
                                    This parameter is set to MC_OVER_TEMP or MC_NO_ERROR */

  uint16_t hLowPassFilterBw;   /**< used to configure the first order software filter bandwidth.
                                    hLowPassFilterBw = NTC_CalcBusReading
                                    call rate [Hz]/ FilterBandwidth[Hz] */
  uint16_t hOverTempThreshold; /**< Represents the over voltage protection intervention threshold.
                                    This parameter is expressed in u16Celsius through formula:
                                    hOverTempThreshold =
                                    (V0[V]+dV/dT[V/°C]*(OverTempThreshold[°C] - T0[°C]))* 65536 / MCU supply voltage */
  uint16_t hOverTempDeactThreshold; /**< Temperature threshold below which an active over temperature fault is cleared.
                                         This parameter is expressed in u16Celsius through formula:
                                         hOverTempDeactThreshold =
                                         (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C] - T0[°C]))* 65536 / MCU supply voltage*/
  int16_t hSensitivity;        /**< NTC sensitivity
                                    This parameter is equal to MCU supply voltage [V] / dV/dT [V/°C] */
  uint32_t wV0;                /**< V0 voltage constant value used to convert the temperature into Volts.
                                    This parameter is equal V0*65536/MCU supply
                                    Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */
  uint16_t hT0;                /**< T0 temperature constant value used to convert the temperature into Volts
                                    Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */
  uint8_t bConvHandle;            /*!< handle to the regular conversion */


  LookupTableHandle_t * pNTCLookupTable;   /* Lookup table handle with NTC data (NTC digital voltage to expected degree Celcius) */

  bool *OutsideTable;  /* Will be set to true if the current temp value is outside the table so it defaults to the nearest value */

} NTCTempSensorHandle_t;


/**
 * 	@brief Initializes temperature sensing conversions
 * 	@param pHandle : Pointer on Handle structure of TemperatureSensor component
 */
void NTCTempSensor_Init(NTCTempSensorHandle_t * pHandle);

/**
 * 	@brief Initializes internal average temperature computed value
 *  @param pHandle : Pointer on Handle structure of TemperatureSensor component
 */
void NTCTempSensor_Clear(NTCTempSensorHandle_t * pHandle);

/**
 * 	@brief Performs the temperature sensing average computation after an ADC conversion
 *  @param pHandle : Pointer on Handle structure of TemperatureSensor component
 *  @retval Fault status : Error reported in case of an over temperature detection
 */
uint16_t NTCTempSensor_CalcAvTemp(NTCTempSensorHandle_t * pHandle);

/**
  * @brief  Returns latest averaged temperature measured expressed in u16Celsius
  * @param 	pHandle : Pointer on Handle structure of TemperatureSensor component
  * @retval AverageTemperature : Current averaged temperature measured (in u16Celsius)
  */
uint16_t NTCTempSensor_GetAvTempDigital(NTCTempSensorHandle_t * pHandle);

/**
  * @brief  Returns latest averaged temperature expressed in Celsius degrees
  * @param 	pHandle : Pointer on Handle structure of TemperatureSensor component
  * @retval AverageTemperature : Latest averaged temperature measured (in Celsius degrees)
  */
int16_t NTCTempSensor_GetAvTempCelcius(NTCTempSensorHandle_t * pHandle);

/**
 * @brief  Returns Temperature measurement fault status
 * Fault status can be either MC_OVER_TEMP when measure exceeds the protection threshold values or
 * MC_NO_ERROR if it is inside authorized range.
 * @param pHandle: Pointer on Handle structure of TemperatureSensor component.
 * @retval Fault status : read internal fault state
 */
uint16_t NTCTempSensor_GetFaultState(NTCTempSensorHandle_t * pHandle);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __NTC_TEMPERATURESENSOR_H */
