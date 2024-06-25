/**
  * @file    ntc_temperature_sensor.c
  * @brief   This file provides firmware functions that implement the features
  *          of the Temperature Sensor component of the Motor Control application.
*/

#include "ntc_temperature_sensor.h"
#include "drive_parameters.h"
#include "motor_signal_processing.h"


/* global Variables -----------------------------------------------------------*/

#define TIMER_STEP          1       /* Timer value increases by 1 */
#define INIT_IGNORE_TIMER   1000    /* Timer for ignoring first NTC values in initialization */
#define DISC_DIGITAL        60000   /* NTC digital value at which temp sensor disconnects at */


/* the minimum acceptable value for NTC - any lower value means 
sensor is disconnected or temeprature is very low
outuput voltage of 10K (resisotr on PCB) || 200K (NTC at -30degrees) on 3.3V = 0.157
So, 200 assumed with a safeguard as the minimum acceptable digital value for NTC, multiplied by 16
Link to calculations: https://docs.google.com/spreadsheets/d/1Hm1R5MGJHDdYDkZ9kOQ2_IeC9xUJphte0QKUo2zZfmU/edit#gid=155823673 */
#define MIN_NTC_FREEZE      3200
// ========================================================================= //

/* Private function prototypes -----------------------------------------------*/
NTCTempFaultStates_t NTC_SetFaultState(NTCTempSensorHandle_t * pHandle);


// ========================================================================= //

/**
  * @brief Returns fault when temperature exceeds the over voltage protection threshold
  *
  *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  *  @r Fault status : Updated internal fault status
  */
NTCTempFaultStates_t NTC_SetFaultState(NTCTempSensorHandle_t * pHandle)
{
    NTCTempFaultStates_t hFault;

    if (pHandle->hAvTempCelcius >= pHandle->hOverTempThreshold)
    {
        hFault = NTC_OT;
    }
    else if (pHandle->hAvTempCelcius >= pHandle->hFoldbackStartTemp)
    {
        hFault = NTC_FOLDBACK;
    }
    else if (pHandle->hAvTempDigital >= DISC_DIGITAL) //disconnection when temp is at lowest value in NTC table
    {
        hFault = NTC_DISC;
    }
    else if (pHandle->hAvTempDigital <= MIN_NTC_FREEZE)
    {
        hFault = NTC_FREEZE;
    }
    else if (pHandle->hAvTempCelcius < pHandle->hOverTempDeactThreshold)
    {
        hFault = NTC_NO_ERRORS;
    }
    else
    {
        hFault = pHandle->hFaultState;
    }
    return hFault;
}

/* Functions ---------------------------------------------------- */


void NTCTempSensor_Init(NTCTempSensorHandle_t * pHandle, NTCTempSensorHandle_t NTCInit, uint16_t defaultTemp)
{
    
    pHandle->bSensorType = NTCInit.bSensorType;
    pHandle->bSensorMixed = NTCInit.bSensorMixed;
    pHandle->hOverTempThreshold = NTCInit.hOverTempDeactThreshold;
    pHandle->hOverTempDeactThreshold = NTCInit.hOverTempThreshold;
    pHandle->hFoldbackStartTemp = NTCInit.hFoldbackStartTemp;
    pHandle->hNTCBetaCoef = NTCInit.hNTCBetaCoef;
    pHandle->hNTCResCoef = NTCInit.hNTCResCoef;
    pHandle->bNTCSource = NTCInit.bNTCSource;
    
    if (pHandle->bSensorType == REAL_SENSOR)
    {
        pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->TempRegConv);  // Need to be register with RegularConvManager
        pHandle->hTimer = INIT_IGNORE_TIMER;
        NTCTempSensor_Clear(pHandle, defaultTemp);
    }
    else  // NO_SENSOR
    {
        pHandle->hFaultState = NTC_NO_ERRORS;
        pHandle->hAvTempDigital = pHandle->hExpectedTempDigital;
    }
}


void NTCTempSensor_Clear(NTCTempSensorHandle_t * pHandle, uint16_t defaultTemp)
{
    pHandle->hAvTempDigital = defaultTemp;
}

uint16_t NTCTempSensor_CalcAvTemp(NTCTempSensorHandle_t * pHandle)
{
    uint32_t wtemp;  // temporary 32 bit variable for calculation
    uint16_t hAux;   // temporary 16 bit variable for calculation
    if (pHandle->bSensorType == REAL_SENSOR)  // Checks if the sensor is real or virtual
    {
        //does have the current motor mixed signal?
        if ((isMotorMixedSignal() == true) && (getMixedSignalRegConvIndex() == pHandle->bConvHandle))
        { 
            hAux = getExtractedMotorTemperature();
        }
        else
        {
            hAux = RegConvMng_ReadConv(pHandle->bConvHandle);   // Reads raw value of converted ADC value.
        }
        
        // Checks for max reading, if yes, no point of averaging
        // Performs first order averaging:
        // new_average = (instantenous_measurment + (previous_average * number_of_smaples - 1)) / number_of_smaples
        wtemp =  (uint32_t)(pHandle->hLowPassFilterBw) - 1u;
        wtemp *= (uint32_t) (pHandle->hAvTempDigital);
        wtemp += hAux;
        wtemp /= (uint32_t)(pHandle->hLowPassFilterBw);
        pHandle->hAvTempDigital = (uint16_t) wtemp;
    }
    else  // NO_SENSOR
    {
        pHandle->hFaultState = NTC_NO_ERRORS;
    }
    
    int32_t wTemp;  // temporary 32 bit variable for calculation
    if((pHandle->bSensorType == REAL_SENSOR) && (pHandle->bNTCSource == MOTOR_NTC)) // Checks for sensor type and sensor channel
    {
        wTemp = NTCTempSensor_CalcMotorTemp(pHandle ,pHandle->hAvTempDigital);
    }
    else if ((pHandle->bSensorType == REAL_SENSOR) && (pHandle->bNTCSource == HEATSINK_NTC))  // Checks for sensor type and sensor channel
    {
        wTemp = NTCTempSensor_CalcHeatSinkTemp (pHandle , pHandle->hAvTempDigital);
    }
    else
    {
        wTemp = pHandle->hExpectedTempCelcius;
    }
    
    pHandle->hAvTempCelcius = (int16_t) wTemp;
    
    pHandle->hFaultState = NTC_SetFaultState(pHandle);  // Retain state
    
    //ignore the first few values after initialization to avoid triggering the error at the beginning
    if (pHandle->hTimer > 0)
    {
        pHandle->hTimer -= TIMER_STEP;
        pHandle->hFaultState = NTC_NO_ERRORS;
    }
    
    return pHandle->hFaultState;
}


uint16_t NTCTempSensor_GetAvTempDigital(NTCTempSensorHandle_t * pHandle)
{
    return pHandle->hAvTempDigital;
}

int16_t NTCTempSensor_GetAvTempCelcius(NTCTempSensorHandle_t * pHandle)
{
    return pHandle->hAvTempCelcius;
}


uint16_t NTCTempSensor_GetFaultState(NTCTempSensorHandle_t * pHandle)
{
    return pHandle->hFaultState;
}

//Calculate the motor temperature from the input ADC data.
uint16_t NTCTempSensor_CalcMotorTemp(NTCTempSensorHandle_t * pHandle, int32_t wInputdata)
{
    double wtemp = 0;

    // Convert ADC input data to voltage
    wtemp = (double)wInputdata / ADC_BYTE_TO_TICS;
    wtemp *= ADC_REFERENCE_VOLTAGE;
    wtemp /= ADC_MAXIMUM_VALUE;

    // Calculate the resistance of the NTC thermistor
    wtemp = (MOTOR_NTC_PULLUP_RESISTOR * wtemp) / (ADC_REFERENCE_VOLTAGE - wtemp);
    wtemp -= MOTOR_NTC_SERIES_PULLDOWN_RESISTOR;

    // Apply Beta coefficient method to calculate temperature
    wtemp *= (pHandle->hNTCResCoef  / 100);
    wtemp = log(wtemp);
    wtemp = pHandle->hNTCBetaCoef / wtemp;

    // Convert temperature from Kelvin to Celsius
    wtemp -= CELSIUS_TO_KELVIN;

    // Return the calculated temperature as an unsigned 16-bit integer
    return (uint16_t)wtemp;
}

//Calculate the heat sink temperature from the input ADC data.
uint16_t NTCTempSensor_CalcHeatSinkTemp(NTCTempSensorHandle_t * pHandle,int32_t wInputdata)
{
    double wtemp = 0;

    // Convert ADC input data to voltage
    wtemp = (double)wInputdata / ADC_BYTE_TO_TICS;

    // Calculate the resistance of the NTC thermistor
    wtemp = (HEATSINK_NTC_PULLDOWN_RESISTOR * ADC_MAXIMUM_VALUE) / wtemp;
    wtemp -= HEATSINK_NTC_PULLDOWN_RESISTOR;

    // Apply Beta coefficient method to calculate temperature
    wtemp *= pHandle->hNTCResCoef;
    wtemp = log(wtemp);
    wtemp = pHandle->hNTCBetaCoef / wtemp;

    // Convert temperature from Kelvin to Celsius
    wtemp -= CELSIUS_TO_KELVIN;

    // Adjust temperature for NTC drift characteristics
    wtemp = wtemp - (wtemp * HEATSINK_NTC_DRIFT_SLOPE - HEATSINK_NTC_DRIFT_INTERCEPT);

    // Return the calculated temperature as an unsigned 16-bit integer
    return (uint16_t)wtemp;
}

