/**
  ******************************************************************************
  * @file    pedal_torque_sensor.c
  * @author  FTEX Inc
  * @brief   This file defines the functions used in higher 
  *          level modules for pedal assist
  ******************************************************************************
*/

#include "pedal_torque_sensor.h"
#include "ASSERT_FTEX.h"

// ==================== Public function prototypes ======================== //


/**
	Pedal torque Sensor conversion Initialization
*/
void PedalTorqSensor_Init(PedalTorqSensorHandle_t * pHandle)
{	
    SignalFiltering_Init(&pHandle->TorqSensorFilter);
    SignalFiltering_ConfigureButterworthFOLP(&pHandle->TorqSensorFilter,
                                                pHandle->hParameters.fFilterAlpha,
                                                    pHandle->hParameters.fFilterBeta);  
    pHandle->bConvHandle = RegConvMng_RegisterRegConv(&pHandle->PTSRegConv);
    PedalTorqSensor_Clear(pHandle);
}


/**
	Pedal torque Sensor ADC hardware values clear
*/
void PedalTorqSensor_Clear(PedalTorqSensorHandle_t * pHandle)
{
    pHandle->hAvTorqueValue = 0u;
    pHandle->hAvADCValue = 0u;
}

/**
	Pedal torque Sensor ADC value calculation and filtering
*/
void PedalTorqSensor_CalcAvValue(PedalTorqSensorHandle_t * pHandle)
{
  uint32_t wAux;
  uint16_t hAux;
  uint16_t hBandwidth;
    
  /* Use the Read conversion Manager for ADC read*/
  hAux = RegConvMng_ReadConv(pHandle->bConvHandle);
  pHandle->hInstTorque = hAux;
  /* Select the filter coefficient based on start or stop condition*/
  if (pHandle->hInstTorque > pHandle->hAvADCValue)
    hBandwidth = pHandle->hParameters.hLowPassFilterBW1;
  else
    hBandwidth = pHandle->hParameters.hLowPassFilterBW2;
  /* Check if the variable not exceeding the limit*/
  if ( hAux != 0xFFFFu )
  {
    wAux =  ( uint32_t )( hBandwidth - 1u ); // Affect Bandwidth to the output value
    wAux *= ( uint32_t ) ( pHandle->hAvADCValue ); // Multiply the Avrg value with the coefficient
    wAux += hAux;
    wAux /= ( uint32_t )( hBandwidth );// Devide the output value  with the coefficient for a new avrg value
    /* Affect the average value to the hAvADCValue */
    pHandle->hAvADCValue = ( uint16_t ) wAux;
  }
  
  /* Compute torque sensor value (between 0 and 65535) */
  hAux = (pHandle->hAvADCValue > pHandle->hParameters.hOffsetPTS) ? 
					(pHandle->hAvADCValue - pHandle->hParameters.hOffsetPTS) : 0; //Substraction without overflow
	
  wAux = (uint32_t)(pHandle->hParameters.bSlopePTS * hAux);
  wAux /= pHandle->hParameters.bDivisorPTS;
  if (wAux > UINT16_MAX) 
  {	
		wAux = UINT16_MAX;
  }
  hAux = (uint16_t)wAux;
	
  pHandle->hAvTorqueValue = hAux;	
}


/**
	Pedal torque Sensor return ADC value
*/
uint16_t PedalTorqSensor_GetAvValue(PedalTorqSensorHandle_t * pHandle)
{
  return pHandle->hAvTorqueValue;
}

/**
	Pedal torque Sensor Reset ADC value
*/
void   PedalTorqSensor_ResetAvValue(PedalTorqSensorHandle_t * pHandle)
{
    pHandle->hAvTorqueValue = 0;
}

/**
	Pedal torque Sensor  Convert Torque sensor data to motor torque
*/
int16_t PedalTorqSensor_ToMotorTorque(PedalTorqSensorHandle_t * pHandle)
{
	int32_t tAux;
	
	/* Compute torque value (between -32768 and 32767) */
	tAux = (int32_t)(pHandle->hAvTorqueValue - pHandle->hParameters.hOffsetMT);
	if (tAux < 0)
  {
		tAux = 0;
	}
  /* Use slope factor to translate the torque speed sensor to a motor torque */
	tAux = ((pHandle->hParameters.bSlopeMT)*tAux);
	tAux /= pHandle->hParameters.bDivisorMT;
	
	/* Data limitation secure if there is any exceed */
	if (tAux > INT16_MAX)
  {
		tAux = INT16_MAX;
  }
  else if (tAux < INT16_MIN)
	{
        tAux = INT16_MIN;
	}
	return (int16_t)tAux;
}
