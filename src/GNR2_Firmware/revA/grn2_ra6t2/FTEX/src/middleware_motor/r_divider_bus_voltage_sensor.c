/**
* @file    r_divider_bus_voltage_sensor.c
* @brief   This file provides firmware functions that implement the  features
*          of the Resistor Divider Bus Voltage Sensor component of the Motor Control SDK:
*
*/

#include "r_divider_bus_voltage_sensor.h"
#include "regular_conversion_manager.h"

/**
* It initializes bus voltage conversion (ADC, ADC channel, conversion time. It must be called only after PWMC_Init.
*/
void ResDivVbusSensor_Init( ResDivVbusSensorHandle_t * pHandle )
{
    pHandle->bConvHandle = RCM_RegisterRegConv(&pHandle->VbusRegConv);  // Need to be register with RegularConvManager    
    ResDivVbusSensor_Clear( pHandle );  // Check
}

/**
* It clears bus voltage FW variable containing average bus voltage value
*/
void ResDivVbusSensor_Clear( ResDivVbusSensorHandle_t * pHandle )
{
    uint16_t aux;  // Temporary data variable
    uint16_t bIndex;  // Temporary bIndex variable
    aux = ( pHandle->hOverVoltageThreshold + pHandle->hUnderVoltageThreshold ) / 2u;
    for ( bIndex = 0u; bIndex < pHandle->hLowPassFilterBw; bIndex++ )
    {
        pHandle->aBuffer[bIndex] = aux;  // It initalizes average buffer elements by average of under and over voltage threshold. 
    }
    pHandle->Super.hLatestConv = aux;
    pHandle->Super.hAvBusVoltageDigital = aux;
    pHandle->bIndex = 0;
}

/**
* It filters measured value of converted bus voltage 
*/
static uint16_t RVBS_ConvertVbusFiltrered( ResDivVbusSensorHandle_t * pHandle )
{
    uint16_t hAux;
    uint8_t vindex;
    uint16_t max = 0, min = 0;
    uint32_t tot = 0u;
    for ( vindex = 0; vindex < pHandle->hLowPassFilterBw; )  // for every elemnet of average filter buffer
    {
        hAux = RCM_ReadConv(pHandle->bConvHandle);
        if ( hAux != 0xFFFFu )  // Checks if measurement is saturated.
        {
            if ( vindex == 0 )  // for first element
            {
                min = hAux;
                max = hAux;
            }
            else
            {
                if ( hAux < min )  // Update min if current measurement is below minimum of all measurment
                {
                    min = hAux;
                }
                if ( hAux > max )  // Update max if current measurement is above maximum of all measurment
                {
                    max = hAux;
                }
            }
            vindex++;
            tot += hAux;
        }
    }
    tot -= max;  // Remove measurements that increases standard deviation 
    tot -= min;
    return (uint16_t)(tot/(pHandle->hLowPassFilterBw-2u));
}

/**
* It actually reads the Vbus ADC conversion and updates average value
*/
uint16_t ResDivVbusSensor_CalcAvVbusFilt( ResDivVbusSensorHandle_t * pHandle )
{
    uint32_t wtemp;
    uint16_t hAux;
    uint8_t i;
    hAux = RVBS_ConvertVbusFiltrered( pHandle );
    if ( hAux != 0xFFFF )  // Check if measurment is not saturated.
    {
        pHandle->aBuffer[pHandle->bIndex] = hAux;  // Update average buffer element with current value
        wtemp = 0;
        for ( i = 0; i < pHandle->hLowPassFilterBw; i++ )
        {
            wtemp += pHandle->aBuffer[i];  // Add all the elements of average filter.
        }
        wtemp /= pHandle->hLowPassFilterBw;  // Calculate average of summation
        pHandle->Super.hAvBusVoltageDigital = ( uint16_t )wtemp;
        pHandle->Super.hLatestConv = hAux;  // Update latest conversion to element of RDivider_Handle
        if ( pHandle->bIndex < pHandle->hLowPassFilterBw - 1 )  // Update bIndex to which next conversion will be stored. 
        {
            pHandle->bIndex++;  
        }
        else
        {
            pHandle->bIndex = 0;
        }
    }
    pHandle->Super.hFaultState = ResDivVbusSensor_CheckFaultState( pHandle );
    return ( pHandle->Super.hFaultState );
}

/**
* It actually performes the Vbus ADC conversion and updates average value
*/
uint16_t ResDivVbusSensor_CalcAvVbus( ResDivVbusSensorHandle_t * pHandle )
{
    uint32_t wtemp;
    uint16_t hAux;
    uint8_t i;
    hAux = RCM_ReadConv(pHandle->bConvHandle); // Reads current value of bus voltage
    if ( hAux != 0xFFFF )  // Checks if measured value is not saturated
    {
        pHandle->aBuffer[pHandle->bIndex] = hAux; // Update average buffer element with current value
        wtemp = 0;
        for ( i = 0; i < pHandle->hLowPassFilterBw; i++ )  
        {
            wtemp += pHandle->aBuffer[i];  // Add all the elements of average filter.
        }
        wtemp /= pHandle->hLowPassFilterBw;  // Calculate average of summation
        pHandle->Super.hAvBusVoltageDigital = ( uint16_t )wtemp;
        pHandle->Super.hLatestConv = hAux;  // Update latest conversion to element of RDivider_Handle
        if ( pHandle->bIndex < pHandle->hLowPassFilterBw - 1 )  // Update bIndex to which next conversion will be stored.
        {
            pHandle->bIndex++;
        }
        else
        {
            pHandle->bIndex = 0;
        }
    }
    pHandle->Super.hFaultState = ResDivVbusSensor_CheckFaultState( pHandle );
    return ( pHandle->Super.hFaultState );
}

/**
* It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on bus voltage and protection threshold values
*/
uint16_t ResDivVbusSensor_CheckFaultState( ResDivVbusSensorHandle_t * pHandle )
{
    uint16_t fault;
    if ( pHandle->Super.hAvBusVoltageDigital > pHandle->hOverVoltageThreshold )  // Checks if measured average voltage is above over voltage threshold
    {
        fault = MC_OVER_VOLT;
    }
    else if ( pHandle->Super.hAvBusVoltageDigital < pHandle->hUnderVoltageThreshold ) // Checks if measured average voltage is below under voltage threshold
    {
        fault = MC_UNDER_VOLT;
    }
    else
    {
        fault = MC_NO_ERROR;
    }
    return fault; // Return fault status, if any
}
