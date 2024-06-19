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

/** NTC Errors Definitions */

typedef enum
{
  NTC_NO_ERRORS   = 0,        /**< No temp related errors or warnings*/
  
  NTC_OT          = 1,        /**< Over temperature error */
  
  NTC_FREEZE      = 2,        /**< Under temperature error */
  
  NTC_FOLDBACK    = 3,        /**< Foldback has started, to trigger foldback warning */
  
  NTC_DISC        = 4,        /**< NTC is disconnected */
  
} NTCTempFaultStates_t;
  
typedef struct
{
    SensorType_t  bSensorType;           /**< Type of instanced temperature.
                                         This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

    bool          bSensorMixed;

    RegConv_t      TempRegConv;

    uint16_t hAvTempDigital;             /**< It contains latest available average temperature.
                                         This parameter is expressed in u16Celsius */
    int16_t hAvTempCelcius;              /**< It contains latest computation of temperature in Celcius */

    uint16_t hExpectedTempDigital;       /**< Default set when no sensor available (ie virtual sensor) */

    uint16_t hExpectedTempCelcius;       /**< Default value when no sensor available (ie virtual sensor).
                                         This parameter is expressed in Celsius */

    NTCTempFaultStates_t hFaultState;    /**< Contains latest Fault code.
                                         This parameter is set to MC_OVER_TEMP or MC_NO_FAULT */

    uint16_t hLowPassFilterBw;           /**< used to configure the first order software filter bandwidth.
                                         hLowPassFilterBw = NTC_CalcBusReading call rate [Hz]/ FilterBandwidth[Hz] */
    int16_t hOverTempThreshold;          /**< Represents the over voltage protection intervention threshold.
                                         This parameter is expressed in degC */
    int16_t hOverTempDeactThreshold;     /**< Temperature threshold below which an active over temperature fault is cleared.
                                         This parameter is expressed in degC */
    int16_t hFoldbackStartTemp;          /**< Temperature at which the foldback starts.
                                         This parameter is expressed in degC */
                                           
    uint8_t bConvHandle;                 /**< handle to the regular conversion */

    uint16_t hTimer;                     /**< timer value used to ignore first values in initialization and to check for sensor disconnections */
    
    uint16_t hNTCBetaCoef;               // Beta coefficient of the NTC thermistor, used for temperature calculation
    
    float    hNTCResCoef;                // This value is calculated based on this formula: exp(NTCBetaCoef / TEMP_25_CELSIUS_IN_KELVIN) / NTC Rated Resistance.
    
    uint8_t  bNTCSource;                 // Source identifier for the NTC sensor, indicating which NTC sensor is being used
  
} NTCTempSensorHandle_t;


/**
 *     @brief Initializes temperature sensing conversions
 *     @param pHandle : Pointer on Handle structure of TemperatureSensor component
 */
void NTCTempSensor_Init(NTCTempSensorHandle_t * pHandle, NTCTempSensorHandle_t NTCInit, uint16_t defaultTemp);
/**
 *     @brief Initializes internal average temperature computed value
 *  @param pHandle : Pointer on Handle structure of TemperatureSensor component
 */
void NTCTempSensor_Clear(NTCTempSensorHandle_t * pHandle, uint16_t defaultTemp);

/**
 *     @brief Performs the temperature sensing average computation after an ADC conversion
 *  @param pHandle : Pointer on Handle structure of TemperatureSensor component
 *  @retval Fault status : Error reported in case of an over temperature detection
 */
uint16_t NTCTempSensor_CalcAvTemp(NTCTempSensorHandle_t * pHandle);

/**
  * @brief  Returns latest averaged temperature measured expressed in u16Celsius
  * @param     pHandle : Pointer on Handle structure of TemperatureSensor component
  * @retval AverageTemperature : Current averaged temperature measured (in u16Celsius)
  */
uint16_t NTCTempSensor_GetAvTempDigital(NTCTempSensorHandle_t * pHandle);

/**
  * @brief  Returns latest averaged temperature expressed in Celsius degrees
  * @param     pHandle : Pointer on Handle structure of TemperatureSensor component
  * @retval AverageTemperature : Latest averaged temperature measured (in Celsius degrees)
  */
int16_t NTCTempSensor_GetAvTempCelcius(NTCTempSensorHandle_t * pHandle);

/**
 * @brief  Returns Temperature measurement fault status
 * Fault status can be either MC_OVER_TEMP when measure exceeds the protection threshold values or
 * MC_NO_FAULT if it is inside authorized range.
 * @param pHandle: Pointer on Handle structure of TemperatureSensor component.
 * @retval Fault status : read internal fault state
 */
uint16_t NTCTempSensor_GetFaultState(NTCTempSensorHandle_t * pHandle);

/**
 * @brief Calculate the motor temperature from the input ADC data.
 * 
 * This function converts the ADC input data to a temperature reading 
 * based on the characteristics of an NTC thermistor. The calculation 
 * uses the Beta coefficient method to determine the temperature.
 * 
 * @param pHandle: Pointer on Handle structure of TemperatureSensor component.
 * @param wInputdata The raw ADC input data.
 * @return uint16_t The calculated temperature in Celsius.
 */
uint16_t NTCTempSensor_CalcMotorTemp(NTCTempSensorHandle_t * pHandle, int32_t wInputdata);

/**
 * @brief Calculate the heat sink temperature from the input ADC data.
 * This function converts the ADC input data to a temperature reading 
 * based on the characteristics of an NTC thermistor. The calculation 
 * uses the Beta coefficient method to determine the temperature.
 * @param pHandle: Pointer on Handle structure of TemperatureSensor component.
 * @param wInputdata The raw ADC input data.
 * @return uint16_t The calculated temperature in Celsius.
 */
uint16_t NTCTempSensor_CalcHeatSinkTemp(NTCTempSensorHandle_t * pHandle, int32_t wInputdata);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __NTC_TEMPERATURESENSOR_H */
