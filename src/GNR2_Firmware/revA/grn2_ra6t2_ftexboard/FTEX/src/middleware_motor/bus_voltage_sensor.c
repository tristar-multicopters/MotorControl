/**
  * @file    bus_voltage_sensor.c
  * @brief   This file provides firmware functions that implement the features
  *          of the BusVoltageSensor component of the Motor Control application.
  *
*/

/* Includes ------------------------------------------------------------------*/

#include "bus_voltage_sensor.h"


uint16_t VbusSensor_GetBusVoltageDigital(BusVoltageSensorHandle_t * pHandle)
{
  return pHandle->hLatestConv;
}


uint16_t VbusSensor_GetAvBusVoltageDigital(BusVoltageSensorHandle_t * pHandle)
{
    return pHandle->hAvBusVoltageDigital;
}


uint16_t VbusSensor_GetAvBusVoltageVolt(BusVoltageSensorHandle_t * pHandle)
{
  uint32_t temp;
  temp = (uint32_t) pHandle->hAvBusVoltageDigital;
  temp *= pHandle->hConversionFactor;  // To convert average voltage from digital to volts scale, AvBusVoltage_V = (AvBusVoltage/ 65526) * (ADC_REF_VOLTAGE / V_BUS_SCALING_FACTOR)
  temp /= 65536u;
  return (uint16_t) temp;
}


uint32_t VbusSensor_GetFaultState(BusVoltageSensorHandle_t * pHandle)
{
  return pHandle->wFaultState;
}
