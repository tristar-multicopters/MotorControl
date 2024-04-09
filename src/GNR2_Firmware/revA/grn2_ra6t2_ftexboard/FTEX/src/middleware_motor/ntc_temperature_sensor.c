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


void NTCTempSensor_Init(NTCTempSensorHandle_t * pHandle, uint16_t defaultTemp)
{
    if (pHandle->bSensorType == REAL_SENSOR)
    {
        if(pHandle->pNTCLookupTable != NULL)
        {
            LookupTable_Init(pHandle->pNTCLookupTable); 
            pHandle->OutsideTable = &(pHandle->pNTCLookupTable->OutsideTable); // Link the OutsideTable flag
        }
        pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->TempRegConv);  // Need to be register with RegularConvManager
        pHandle->hTimer = INIT_IGNORE_TIMER;
        NTCTempSensor_Clear(pHandle, defaultTemp);
    }
    else  // VIRTUAL_SENSOR
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
    else  // VIRTUAL_SENSOR
    {
        pHandle->hFaultState = NTC_NO_ERRORS;
    }
  
    int32_t wTemp;  // temporary 32 bit variable for calculation
    if ((pHandle->bSensorType == REAL_SENSOR) && (pHandle->pNTCLookupTable != NULL))  // Checks for sensor type
    {
        wTemp = LookupTable_CalcOutput(pHandle->pNTCLookupTable, pHandle->hAvTempDigital);          
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

