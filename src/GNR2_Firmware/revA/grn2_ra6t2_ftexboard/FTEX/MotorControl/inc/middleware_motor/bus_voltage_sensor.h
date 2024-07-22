/**
  * @file    bus_voltage_sensor.h
  * @brief   This file contains all definitions and functions prototypes for the
  *          BusVoltageSensor component of the Motor Control application.
*/

#ifndef __BUSVOLTAGESENSOR_H
#define __BUSVOLTAGESENSOR_H

#include "mc_type.h"
#include "pwm_curr_fdbk.h"

// ====== Structure used to configure bus voltage sensor parameters ========== //

typedef struct
{
    SensorType_t SensorType;  // It contains the information about the type of instanced bus voltage sensor object. It can be equal to REAL_SENSOR or NO_SENSOR
    uint16_t hConversionFactor;  // It is used to convert bus voltage from u16Volts into real Volts (V). u16Volt = 65536/hConversionFactor Volts
                                // For real sensors hConversionFactor it's equal to the product between the expected MCU voltage and the voltage sensing network
                                // attenuation. For virtual sensors it must be equal to 500
    uint16_t hLatestConv;  // It contains latest Vbus converted value expressed in u16Volts format
    uint16_t hAvBusVoltageDigital;  // It contains latest available average Vbus expressed in digit
    uint32_t wFaultState;  // It contains latest Fault code (MC_NO_ERROR, MC_OVER_VOLT or MC_UNDER_VOLT)
} BusVoltageSensorHandle_t;

// ==================== Public function prototypes ========================= //

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  It return latest Vbus conversion result expressed in u16Volt
  * @param  pHandle related Handle of BusVoltageSensorHandle_t
  * @retval uint16_t Latest Vbus conversion result in digit
  */
uint16_t VbusSensor_GetBusVoltageDigital(BusVoltageSensorHandle_t * pHandle);

/**
  * @brief  It return latest averaged Vbus measurement expressed in u16Volt
  * @param  pHandle related Handle of BusVoltageSensorHandle_t
  * @retval uint16_t Latest averaged Vbus measurement in digit
  */
uint16_t VbusSensor_GetAvBusVoltageDigital(BusVoltageSensorHandle_t * pHandle);

/**
  * @brief  It return latest averaged Vbus measurement expressed in Volts
  * @param  pHandle related Handle of BusVoltageSensorHandle_t
  * @retval uint16_t Latest averaged Vbus measurement in Volts
  */
uint16_t VbusSensor_GetAvBusVoltageVolt(BusVoltageSensorHandle_t * pHandle);

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_FAULT depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related Handle of BusVoltageSensorHandle_t
  * @retval uint16_t Fault code error
  */
uint32_t VbusSensor_GetFaultState(BusVoltageSensorHandle_t * pHandle);

#endif
