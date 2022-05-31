/**
* @file bus_voltage_sensor.c
* @brief This file provides firmware functions that implement the features of the BusVoltageSensor component of the Motor Control SDK.
*/

#include "bus_voltage_sensor.h"

// ========================================================================= //

/**
* It return latest Vbus conversion result expressed in u16Volt
*/
uint16_t VBS_GetBusVoltage_d( BusVoltageSensor_Handle_t * pHandle )
{
    return ( pHandle->LatestConv );
}

/**
* It return latest averaged Vbus measurement expressed in u16Volt
*/
uint16_t VBS_GetAvBusVoltage_d( BusVoltageSensor_Handle_t * pHandle )
{
    return ( pHandle->AvBusVoltage_d );
}

/**
* It return latest averaged Vbus measurement expressed in Volts
*/
uint16_t VBS_GetAvBusVoltage_V( BusVoltageSensor_Handle_t * pHandle )
{
    uint32_t temp;
    temp = ( uint32_t )( pHandle->AvBusVoltage_d );
    temp *= pHandle->ConversionFactor;
    temp /= 65536u;
    return ( ( uint16_t )temp );
}

/**
* It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on bus voltage and protection threshold values
*/
uint16_t VBS_CheckVbus( BusVoltageSensor_Handle_t * pHandle )
{
    return ( pHandle->FaultState );
}

