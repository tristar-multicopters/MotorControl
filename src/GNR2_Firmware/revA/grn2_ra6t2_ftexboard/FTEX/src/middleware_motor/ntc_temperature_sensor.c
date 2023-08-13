/**
  * @file    ntc_temperature_sensor.c
  * @brief   This file provides firmware functions that implement the features
  *          of the Temperature Sensor component of the Motor Control application.
*/

#include "ntc_temperature_sensor.h"

/* global Variables -----------------------------------------------------------*/

/* the minimum acceptable value for NTC - any lower value means 
sensor is disconnected or temeprature is very low
outuput voltage of 10K (resisotr on PCB) || 200K (NTC at -40degrees) on 3.3V = 0.157
So, 200 assumed with a safeguard as the minimum acceptable digital value for NTC*/
const uint8_t MIMIMUM_NTC_FREEZING = 200;
// ========================================================================= //

/* Private function prototypes -----------------------------------------------*/
uint16_t NTC_SetFaultState(NTCTempSensorHandle_t * pHandle);

// ========================================================================= //

/**
  * @brief Returns fault when temperature exceeds the over voltage protection threshold
  *
  *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  *  @r Fault status : Updated internal fault status
  */
uint16_t NTC_SetFaultState(NTCTempSensorHandle_t * pHandle)
{
  uint16_t hFault;

  if (pHandle->hAvTempCelcius > pHandle->hOverTempThreshold)
  {
    hFault = 1;
  }
  else if (pHandle->hAvTempCelcius < pHandle->hOverTempDeactThreshold)
  {
    hFault = 0;
  }
  else
  {
    hFault = pHandle->hFaultState;
  }
  return hFault;
}

/* Functions ---------------------------------------------------- */


void NTCTempSensor_Init(NTCTempSensorHandle_t * pHandle)
{
  if (pHandle->bSensorType == REAL_SENSOR)
  {
      if(pHandle->pNTCLookupTable != NULL)
      {
          LookupTable_Init(pHandle->pNTCLookupTable);
          
          pHandle->OutsideTable = &(pHandle->pNTCLookupTable->OutsideTable); // Link the OutsideTable flag
      }
      pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->TempRegConv);  // Need to be register with RegularConvManager
      NTCTempSensor_Clear(pHandle);
  }
  else  // VIRTUAL_SENSOR
  {
      pHandle->hFaultState = 0;
      pHandle->hAvTempDigital = pHandle->hExpectedTempDigital;
  }
}


void NTCTempSensor_Clear(NTCTempSensorHandle_t * pHandle)
{
  pHandle->hAvTempDigital = 0u;
}


uint16_t NTCTempSensor_CalcAvTemp(NTCTempSensorHandle_t * pHandle)
{
    uint32_t wtemp;  // temporary 32 bit variable for calculation
    uint16_t hAux;   // temporary 16 bit variable for calculation
    if (pHandle->bSensorType == REAL_SENSOR)  // Checks if the sensor is real or virtual
    {
      hAux = RegConvMng_ReadConv(pHandle->bConvHandle);   // Reads raw value of converted ADC value.
      if (hAux != 0xFFFFu)  // Checks for max reading, if yes, no point of averaging
          {   // Performs first order averaging:
              // new_average = (instantenous_measurment + (previous_average * number_of_smaples - 1)) / number_of_smaples
          wtemp =  (uint32_t)(pHandle->hLowPassFilterBw) - 1u;
          wtemp *= (uint32_t) (pHandle->hAvTempDigital);
          wtemp += hAux;
          wtemp /= (uint32_t)(pHandle->hLowPassFilterBw);
          pHandle->hAvTempDigital = (uint16_t) wtemp;
      }
    }
    else  // VIRTUAL_SENSOR
    {
      pHandle->hFaultState = 0;
    }
  
    int32_t wTemp;  // temporary 32 bit variable for calculation
    if (pHandle->bSensorType == REAL_SENSOR && pHandle->pNTCLookupTable != NULL)  // Checks for sensor type
    {
        wTemp = LookupTable_CalcOutput(pHandle->pNTCLookupTable, pHandle->hAvTempDigital);          
    }
    else
    {
        wTemp = pHandle->hExpectedTempCelcius;
    }
    
    pHandle->hAvTempCelcius = (int16_t) wTemp;
    
    pHandle->hFaultState = NTC_SetFaultState(pHandle);  // Retain state
    
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

