/**
* @file    r_divider_bus_voltage_sensor.c
* @brief   This file provides firmware functions that implement the features
*          of the Temperature Sensor component of the Motor Control SDK.
*
*/

#include "ntc_temperature_sensor.h"

// ==================== Private function prototypes ======================== //

uint16_t NTC_SetFaultState( NTC_Handle_t * pHandle );

// ========================================================================= //

/**
* Returns fault when temperature exceeds the over temp protection threshold. This fault is cleared only when measured temperatures goes below over temp deactivation threshold. 
*/
uint16_t NTC_SetFaultState( NTC_Handle_t * pHandle )
{
    uint16_t hFault;
    if ( pHandle->hAvTemp_d > pHandle->hOverTempThreshold )  // Checks if measured temperature is over hOverTempThreshold
    {
        hFault = MC_OVER_TEMP;
    }
    else if ( pHandle->hAvTemp_d < pHandle->hOverTempDeactThreshold ) // Clears the fault only when measured temperature is below deactivation threshold.
    {
        hFault = MC_NO_ERROR;
    }
    else
    {
        hFault = pHandle->hFaultState;
    }
    return hFault;
}

/**
* Initializes temperature sensing conversions
*/
void NTC_Init( NTC_Handle_t * pHandle )
{
    if ( pHandle->bSensorType == REAL_SENSOR )
    {
        pHandle->convHandle = RCM_RegisterRegConv(&pHandle->TempRegConv);  // Need to be register with RegularConvManager
        NTC_Clear( pHandle );
    }
    else  // VIRTUAL_SENSOR
    {
        pHandle->hFaultState = MC_NO_ERROR;
        pHandle->hAvTemp_d = pHandle->hExpectedTemp_d;
    }
}

/**
* Clears internal average temperature computed value
*/
void NTC_Clear( NTC_Handle_t * pHandle )
{
    pHandle->hAvTemp_d = 0u;
}

/**
* Performs the temperature sensing average computation after reading an ADC conversion
*/
uint16_t NTC_CalcAvTemp( NTC_Handle_t * pHandle )
{
    uint32_t wtemp;  // temporary 32 bit variable for calculation 
    uint16_t hAux;   // temporary 16 bit variable for calculation
    if ( pHandle->bSensorType == REAL_SENSOR )  // Checks if the sensor is real or virtual
    {
        hAux = RCM_ReadConv(pHandle->convHandle);   // Reads raw value of converted ADC value.
        if ( hAux != 0xFFFFu )  // Checks for max reading, if yes, no point of averaging
            {   // Performs first order averaging:
                // new_average = (instantenous_measurment + (previous_average * number_of_smaples - 1 )) / number_of_smaples
            wtemp =  ( uint32_t )( pHandle->hLowPassFilterBW ) - 1u;
            wtemp *= ( uint32_t ) ( pHandle->hAvTemp_d );
            wtemp += hAux;
            wtemp /= ( uint32_t )( pHandle->hLowPassFilterBW );
            pHandle->hAvTemp_d = ( uint16_t ) wtemp;
        }
        pHandle->hFaultState = NTC_SetFaultState( pHandle );  // Retain state
    }
    else  // VIRTUAL_SENSOR 
    {
        pHandle->hFaultState = MC_NO_ERROR;
    }
    return pHandle->hFaultState;
}

/**
* Returns latest averaged temperature measured expressed in u16Celsius
*/
uint16_t NTC_GetAvTemp_d( NTC_Handle_t * pHandle )
{
    return pHandle->hAvTemp_d;
}

/**
* Returns latest averaged temperature expressed in Celsius degrees
*/
int16_t NTC_GetAvTemp_C( NTC_Handle_t * pHandle )
{
    int32_t wTemp;  // temporary 32 bit variable for calculation 
    if ( pHandle->bSensorType == REAL_SENSOR )  // Checks for sensor type
        {  // Converts averaged temperature measurement from digital to celsius by formula: 
           // AvTCelsius = (( (AvTDigital-DigitalOffset)*Sensitivity )/MaxDigitalValue)+ CelsiusOffset 
        wTemp = ( int32_t )( pHandle->hAvTemp_d );
        wTemp -= ( int32_t )( pHandle->wV0 );
        wTemp *= pHandle->hSensitivity;
        wTemp = wTemp / 65536 + ( int32_t )( pHandle->hT0 );
    }
    else
    {
        wTemp = pHandle->hExpectedTemp_C;
    }
    return (int16_t) wTemp;
}

/**
* Returns Temperature measurement fault status Fault status can be either MC_OVER_TEMP when measure exceeds the protection threshold values or MC_NO_ERROR if it is inside authorized range.
*/
uint16_t NTC_CheckTemp( NTC_Handle_t * pHandle )
{
    return pHandle->hFaultState;
}
