/**
  * @file    virtual_bus_voltage_sensor.c
  * @brief   This file provides firmware functions that implement the features
  *          of the Virtual Bus Voltage Sensor component of the Motor Control application.
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "virtual_bus_voltage_sensor.h"


void VirtualBusVoltSensor_Init( VirtualBusVoltageSensorHandle_t * pHandle )
{
  pHandle->Super.hFaultState = MC_NO_ERROR;
  pHandle->Super.hLatestConv = pHandle->hExpectedVbusDigital;
  pHandle->Super.hAvBusVoltageDigital = pHandle->hExpectedVbusDigital;
}


void VirtualBusVoltSensor_Clear( VirtualBusVoltageSensorHandle_t * pHandle )
{
  return;
}


uint16_t VirtualBusVoltSensor_NoErrors( VirtualBusVoltageSensorHandle_t * pHandle )
{
  return ( MC_NO_ERROR );
}


/** @} */



